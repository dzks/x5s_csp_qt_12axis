#include "QT/PlateShapeMotionWorker.hpp"

#include "Ethercat/config.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "time_utils.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <exception>

namespace
{
    constexpr double kPi =
        3.14159265358979323846;

    double DegToRad(
        double degree
    )
    {
        return degree * kPi / 180.0;
    }
}

PlateShapeMotionWorker::PlateShapeMotionWorker(
    MotorControl& motor_control,
    ParallelControl& upper_left,
    ParallelControl& upper_right,
    ParallelControl& lower_left,
    ParallelControl& lower_right,
    MultiParallelControl& multi_parallel,
    QObject* parent
)
    : QObject(parent),
      motor_control_(motor_control),
      upper_left_(upper_left),
      upper_right_(upper_right),
      lower_left_(lower_left),
      lower_right_(lower_right),
      multi_parallel_(multi_parallel)
{
    hold_targets_.resize(
        config::kActiveAxisCount,
        0
    );
}

bool PlateShapeMotionWorker::RequestFormInitialShape(
    PlatformScope scope
)
{
    if (motion_busy_)
    {
        return false;
    }

    int expected =
        static_cast<int>(
            MotionCommand::None
        );

    requested_scope_ =
        static_cast<int>(
            scope
        );

    return requested_command_.compare_exchange_strong(
        expected,
        static_cast<int>(
            MotionCommand::FormInitialShape
        )
    );
}

bool PlateShapeMotionWorker::RequestResetToStraight(
    PlatformScope scope
)
{
    if (motion_busy_)
    {
        return false;
    }

    int expected =
        static_cast<int>(
            MotionCommand::None
        );

    requested_scope_ =
        static_cast<int>(
            scope
        );

    return requested_command_.compare_exchange_strong(
        expected,
        static_cast<int>(
            MotionCommand::ResetToStraight
        )
    );
}

void PlateShapeMotionWorker::Stop()
{
    running_ =
        false;

    requested_command_ =
        static_cast<int>(
            MotionCommand::Stop
        );
}

void PlateShapeMotionWorker::Run()
{
    emit StatusTextChanged(
        "状态：等待轴使能..."
    );

    bool enabled =
        motor_control_.WaitAllAxesOperationEnabled(
            100,
            60000
        );

    if (!enabled)
    {
        emit StatusTextChanged(
            "状态：轴使能失败"
        );

        emit MotionFinished(
            false,
            "WaitAllAxesOperationEnabled failed."
        );

        return;
    }

    emit StatusTextChanged(
        "状态：已使能，进入持续保持"
    );

    UpdateHoldTargetsFromActual();

    using Clock =
        std::chrono::steady_clock;

    auto next_time =
        Clock::now();

    while (running_)
    {
        int command_int =
            requested_command_.exchange(
                static_cast<int>(
                    MotionCommand::None
                )
            );

        MotionCommand command =
            static_cast<MotionCommand>(
                command_int
            );

        if (command == MotionCommand::Stop)
        {
            break;
        }

        if (command == MotionCommand::FormInitialShape)
        {
            motion_busy_ =
                true;

            emit MotionBusyChanged(
                true
            );

            emit StatusTextChanged(
                "状态：直板 -> 初始板型"
            );

            PlatformScope scope =
                static_cast<PlatformScope>(
                    requested_scope_.load()
                );

            bool ok =
                ExecutePlateShapeMotion(
                    scope,
                    0.0,
                    1.0
                );

            motion_busy_ =
                false;

            emit MotionBusyChanged(
                false
            );

            emit MotionFinished(
                ok,
                ok
                    ? "直板 -> 初始板型完成"
                    : "直板 -> 初始板型失败"
            );

            emit StatusTextChanged(
                ok
                    ? "状态：成形完成，持续保持"
                    : "状态：成形失败，持续保持"
            );

            next_time =
                Clock::now();
        }
        else if (command == MotionCommand::ResetToStraight)
        {
            motion_busy_ =
                true;

            emit MotionBusyChanged(
                true
            );

            emit StatusTextChanged(
                "状态：初始板型 -> 直板"
            );

            PlatformScope scope =
                static_cast<PlatformScope>(
                    requested_scope_.load()
                );

            bool ok =
                ExecutePlateShapeMotion(
                    scope,
                    1.0,
                    0.0
                );

            motion_busy_ =
                false;

            emit MotionBusyChanged(
                false
            );

            emit MotionFinished(
                ok,
                ok
                    ? "初始板型 -> 直板完成"
                    : "初始板型 -> 直板失败"
            );

            emit StatusTextChanged(
                ok
                    ? "状态：复位完成，持续保持"
                    : "状态：复位失败，持续保持"
            );

            next_time =
                Clock::now();
        }

        next_time +=
            std::chrono::nanoseconds(
                config::kCycleTimeNs
            );

        RunHoldCycle();

        std::this_thread::sleep_until(
            next_time
        );
    }

    emit StatusTextChanged(
        "状态：控制线程结束"
    );
}

void PlateShapeMotionWorker::RunHoldCycle()
{
    EthercatMaster& master =
        motor_control_.Master();

    master.setApplicationTime(
        getMonotonicTimeNs()
    );

    master.receive();
    master.checkState();

    for (int axis_index = 0;
         axis_index < config::kActiveAxisCount;
         ++axis_index)
    {
        X5sAxis& axis =
            motor_control_.Axis(
                axis_index
            );

        axis.read();

        axis.setModeOfOperation(
            config::kModeCSP
        );

        axis.setCspTargetPosition(
            hold_targets_[axis_index]
        );

        axis.write();
    }

    motor_control_.SyncClocks();
    master.send();

    ++ui_emit_counter_;

    if (ui_emit_counter_ >= 50)
    {
        ui_emit_counter_ =
            0;

        EmitCurrentState();
    }
}

bool PlateShapeMotionWorker::ExecutePlateShapeMotion(
    PlatformScope scope,
    double p_start,
    double p_target
)
{
    try
    {
        const int64_t trajectory_cycles =
            5000;

        std::vector<MultiParallelControl::ParallelTrajectoryPlan> plans;

        if (scope == PlatformScope::UpperOnly
            || scope == PlatformScope::UpperAndLower)
        {
            ParallelControl::PlateShapePlanOptions upper_left_options =
                MakeOptions(
                    p_start,
                    p_target,
                    false,
                    false
                );

            ParallelControl::PlateShapePlanOptions upper_right_options =
                MakeOptions(
                    p_start,
                    p_target,
                    true,
                    false
                );

            plans.push_back(
                upper_left_.BuildPlateShapeQuinticPlan(
                    upper_left_options,
                    trajectory_cycles
                )
            );

            plans.push_back(
                upper_right_.BuildPlateShapeQuinticPlan(
                    upper_right_options,
                    trajectory_cycles
                )
            );
        }

        if (scope == PlatformScope::LowerOnly
            || scope == PlatformScope::UpperAndLower)
        {
            ParallelControl::PlateShapePlanOptions lower_left_options =
                MakeOptions(
                    p_start,
                    p_target,
                    false,
                    true
                );

            ParallelControl::PlateShapePlanOptions lower_right_options =
                MakeOptions(
                    p_start,
                    p_target,
                    true,
                    true
                );

            plans.push_back(
                lower_left_.BuildPlateShapeQuinticPlan(
                    lower_left_options,
                    trajectory_cycles
                )
            );

            plans.push_back(
                lower_right_.BuildPlateShapeQuinticPlan(
                    lower_right_options,
                    trajectory_cycles
                )
            );
        }

        if (plans.empty())
        {
            std::cerr
                << "[PlateShapeWorker] empty plans.\n";

            return false;
        }

        bool ok =
            multi_parallel_.ExecuteSynchronizedTrajectories(
                plans,
                2000,
                200
            );

        if (ok)
        {
            UpdateHoldTargetsFromPlans(
                plans
            );
        }
        else
        {
            UpdateHoldTargetsFromActual();
        }

        EmitCurrentState();

        return ok;
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[PlateShapeWorker] exception: "
            << e.what()
            << "\n";

        UpdateHoldTargetsFromActual();

        return false;
    }
}

void PlateShapeMotionWorker::UpdateHoldTargetsFromActual()
{
    EthercatMaster& master =
        motor_control_.Master();

    master.receive();
    master.checkState();

    for (int axis_index = 0;
         axis_index < config::kActiveAxisCount;
         ++axis_index)
    {
        X5sAxis& axis =
            motor_control_.Axis(
                axis_index
            );

        axis.read();

        hold_targets_[axis_index] =
            axis.actualPosition();
    }
}

void PlateShapeMotionWorker::UpdateHoldTargetsFromPlans(
    const std::vector<MultiParallelControl::ParallelTrajectoryPlan>& plans
)
{
    for (const auto& plan : plans)
    {
        if (plan.count_trajectory.empty())
        {
            continue;
        }

        const trajectory::AxisCount3& final_point =
            plan.count_trajectory.back();

        for (int i = 0;
             i < 3;
             ++i)
        {
            int axis_index =
                plan.axis_indices[i];

            if (axis_index >= 0
                && axis_index < config::kActiveAxisCount)
            {
                hold_targets_[axis_index] =
                    final_point[i];
            }
        }
    }
}

void PlateShapeMotionWorker::EmitCurrentState()
{
    SystemPoseState state{};

    ThreePRR::TargetPose upper_left_pose =
        upper_left_.GetCurrentPose();

    ThreePRR::TargetPose upper_right_pose =
        upper_right_.GetCurrentPose();

    ThreePRR::TargetPose lower_left_pose =
        lower_left_.GetCurrentPose();

    ThreePRR::TargetPose lower_right_pose =
        lower_right_.GetCurrentPose();

    state.upper_left.xQ =
        upper_left_pose.xQ;
    state.upper_left.yQ =
        upper_left_pose.yQ;
    state.upper_left.phi =
        upper_left_pose.phi;

    state.upper_right.xQ =
        upper_right_pose.xQ;
    state.upper_right.yQ =
        upper_right_pose.yQ;
    state.upper_right.phi =
        upper_right_pose.phi;

    state.lower_left.xQ =
        lower_left_pose.xQ;
    state.lower_left.yQ =
        lower_left_pose.yQ;
    state.lower_left.phi =
        lower_left_pose.phi;

    state.lower_right.xQ =
        lower_right_pose.xQ;
    state.lower_right.yQ =
        lower_right_pose.yQ;
    state.lower_right.phi =
        lower_right_pose.phi;

    emit StateUpdated(
        state
    );
}

ParallelControl::PlateShapePlanOptions
PlateShapeMotionWorker::MakeOptions(
    double p_start,
    double p_target,
    bool is_right_side,
    bool is_lower_side
) const
{
    (void)is_lower_side;

    ParallelControl::PlateShapePlanOptions options;

    options.p_start =
        p_start;

    options.p_target =
        p_target;

    options.straight_yQ =
        140.0;

    options.straight_phi =
        DegToRad(
            0.0
        );

    options.left_shape_dy_mm =
        50.0;

    options.right_shape_dy_mm =
        -35.0;

    options.right_shape_dx_mm =
        -7.676;

    options.shape_dphi_rad =
        DegToRad(
            0.0
        );

    if (is_right_side)
    {
        options.straight_xQ =
            1145.0;
    }
    else
    {
        options.straight_xQ =
            450.0;
    }

    return options;
}