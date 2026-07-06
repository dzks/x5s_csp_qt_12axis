#include "Ethercat/config.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/pdo_config.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "time_utils.hpp"

#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"

#include "Motor/Motor_Control.hpp"
#include "Motor/Parallel_Control.hpp"
#include "Motor/Multi_Parallel_Control.hpp"

#include "Trajectory/count_trajectory_generator.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <thread>
#include <vector>

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

    enum class SimpleCommand
    {
        None = -1,
        FormInitialShape = 0,
        ResetToStraight = 1,
        Stop = 9
    };

    std::atomic<int> g_requested_command{
        static_cast<int>(
            SimpleCommand::None
        )
    };

    std::atomic_bool g_running{
        true
    };

    std::atomic_bool g_busy{
        false
    };

    bool RequestCommand(
        SimpleCommand command
    )
    {
        int expected =
            static_cast<int>(
                SimpleCommand::None
            );

        return g_requested_command.compare_exchange_strong(
            expected,
            static_cast<int>(
                command
            )
        );
    }

    ParallelControl::PlateShapePlanOptions MakeLowerPlateShapeOptions(
        double p_start,
        double p_target,
        bool is_right_side
    )
    {
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

        // 左端：140 -> 190
        options.left_shape_dy_mm =
            50.0;

        // 右端：140 -> 105
        options.right_shape_dy_mm =
            -35.0;

        // 右端水平随动：x -> x - 7.676
        // 如果你实际下侧机构坐标方向相反，把这里改成 +7.676
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

    void UpdateHoldTargetsFromActual(
        MotorControl& motor_control,
        std::vector<int32_t>& hold_targets
    )
    {
        EthercatMaster& master =
            motor_control.Master();

        master.receive();
        master.checkState();

        for (int axis_index = 0;
             axis_index < config::kActiveAxisCount;
             ++axis_index)
        {
            X5sAxis& axis =
                motor_control.Axis(
                    axis_index
                );

            axis.read();

            hold_targets[axis_index] =
                motor_control.GetAxisEncoderCount(
                    axis_index
                );
        }

        std::cout
            << "[Hold] Hold targets updated from actual position.\n";
    }

    void UpdateHoldTargetsFromPlans(
        const std::vector<MultiParallelControl::ParallelTrajectoryPlan>& plans,
        std::vector<int32_t>& hold_targets
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
                    hold_targets[axis_index] =
                        final_point[i];
                }
            }
        }

        std::cout
            << "[Hold] Hold targets updated from final trajectory point.\n";
    }

    void RunHoldCycle(
        MotorControl& motor_control,
        const std::vector<int32_t>& hold_targets
    )
    {
        EthercatMaster& master =
            motor_control.Master();

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
                motor_control.Axis(
                    axis_index
                );

            axis.read();

            axis.setModeOfOperation(
                config::kModeCSP
            );

            axis.setCspTargetPosition(
                hold_targets[axis_index]
            );

            axis.write();
        }

        motor_control.SyncClocks();

        master.send();
    }

    bool ExecuteLowerPlateShapeMotion(
        MultiParallelControl& multi_parallel,
        ParallelControl& lower_left_parallel,
        ParallelControl& lower_right_parallel,
        double p_start,
        double p_target,
        std::vector<int32_t>& hold_targets
    )
    {
        const int64_t trajectory_cycles =
            5000;

        std::vector<MultiParallelControl::ParallelTrajectoryPlan> plans;

        ParallelControl::PlateShapePlanOptions lower_left_options =
            MakeLowerPlateShapeOptions(
                p_start,
                p_target,
                false
            );

        ParallelControl::PlateShapePlanOptions lower_right_options =
            MakeLowerPlateShapeOptions(
                p_start,
                p_target,
                true
            );

        std::cout
            << "[Motion] Building lower-left plate shape plan...\n";

        plans.push_back(
            lower_left_parallel.BuildPlateShapeQuinticPlan(
                lower_left_options,
                trajectory_cycles
            )
        );

        std::cout
            << "[Motion] Building lower-right plate shape plan...\n";

        plans.push_back(
            lower_right_parallel.BuildPlateShapeQuinticPlan(
                lower_right_options,
                trajectory_cycles
            )
        );

        std::cout
            << "[Motion] Executing lower plate shape motion...\n";

        bool ok =
            multi_parallel.ExecuteSynchronizedTrajectories(
                plans,
                2000,
                200
            );

        if (ok)
        {
            UpdateHoldTargetsFromPlans(
                plans,
                hold_targets
            );

            std::cout
                << "[Motion] Lower plate shape motion success.\n";
        }
        else
        {
            std::cerr
                << "[Motion] Lower plate shape motion failed.\n";
        }

        return ok;
    }

    void MotorThreadMain(
        MotorControl& motor_control,
        ParallelControl& upper_left_parallel,
        ParallelControl& upper_right_parallel,
        ParallelControl& lower_left_parallel,
        ParallelControl& lower_right_parallel,
        MultiParallelControl& multi_parallel
    )
    {
        (void)upper_left_parallel;
        (void)upper_right_parallel;

        std::vector<int32_t> hold_targets(
            config::kActiveAxisCount,
            0
        );

        std::cout
            << "[MotorThread] Waiting all axes Operation Enabled...\n";

        bool enabled =
            motor_control.WaitAllAxesOperationEnabled(
                100,
                60000
            );

        if (!enabled)
        {
            std::cerr
                << "[MotorThread] WaitAllAxesOperationEnabled failed.\n";

            g_running =
                false;

            return;
        }

        std::cout
            << "[MotorThread] All axes enabled.\n";

        UpdateHoldTargetsFromActual(
            motor_control,
            hold_targets
        );

        using Clock =
            std::chrono::steady_clock;

        auto next_time =
            Clock::now();

        while (g_running)
        {
            int command_value =
                g_requested_command.exchange(
                    static_cast<int>(
                        SimpleCommand::None
                    )
                );

            SimpleCommand command =
                static_cast<SimpleCommand>(
                    command_value
                );

            if (command == SimpleCommand::Stop)
            {
                std::cout
                    << "[MotorThread] Stop command received.\n";

                g_running =
                    false;

                break;
            }

            if (command == SimpleCommand::FormInitialShape)
            {
                g_busy =
                    true;

                std::cout
                    << "[MotorThread] Command 0: Lower straight -> initial shape.\n";

                bool ok =
                    ExecuteLowerPlateShapeMotion(
                        multi_parallel,
                        lower_left_parallel,
                        lower_right_parallel,
                        0.0,
                        1.0,
                        hold_targets
                    );

                if (!ok)
                {
                    UpdateHoldTargetsFromActual(
                        motor_control,
                        hold_targets
                    );
                }

                g_busy =
                    false;

                next_time =
                    Clock::now();
            }
            else if (command == SimpleCommand::ResetToStraight)
            {
                g_busy =
                    true;

                std::cout
                    << "[MotorThread] Command 1: Lower initial shape -> straight.\n";

                bool ok =
                    ExecuteLowerPlateShapeMotion(
                        multi_parallel,
                        lower_left_parallel,
                        lower_right_parallel,
                        1.0,
                        0.0,
                        hold_targets
                    );

                if (!ok)
                {
                    UpdateHoldTargetsFromActual(
                        motor_control,
                        hold_targets
                    );
                }

                g_busy =
                    false;

                next_time =
                    Clock::now();
            }

            next_time +=
                std::chrono::nanoseconds(
                    config::kCycleTimeNs
                );

            RunHoldCycle(
                motor_control,
                hold_targets
            );

            std::this_thread::sleep_until(
                next_time
            );
        }

        std::cout
            << "[MotorThread] Exit.\n";
    }
}

int main(
    int argc,
    char* argv[]
)
{
    (void)argc;
    (void)argv;

    EthercatMaster master;

    if (!master.init())
    {
        std::cerr
            << "EtherCAT init failed.\n";

        return 1;
    }

    std::cout
        << "EtherCAT init success.\n";

    std::vector<X5sAxis> axes;

    axes.reserve(
        config::kActiveAxisCount
    );

    for (int axis = 0;
         axis < config::kActiveAxisCount;
         ++axis)
    {
        axes.emplace_back(
            master.domainData(),
            &g_x5s_offsets[axis],
            axis
        );
    }

    MotorControl motor_control(
        master,
        axes
    );

    ParallelControl upper_left_parallel(
        motor_control,
        ParallelSide::UpperLeft
    );

    ParallelControl upper_right_parallel(
        motor_control,
        ParallelSide::UpperRight
    );

    ParallelControl lower_left_parallel(
        motor_control,
        ParallelSide::LowerLeft
    );

    ParallelControl lower_right_parallel(
        motor_control,
        ParallelSide::LowerRight
    );

    MultiParallelControl multi_parallel(
        motor_control,
        upper_left_parallel,
        upper_right_parallel,
        lower_left_parallel,
        lower_right_parallel
    );

    std::thread motor_thread(
        MotorThreadMain,
        std::ref(motor_control),
        std::ref(upper_left_parallel),
        std::ref(upper_right_parallel),
        std::ref(lower_left_parallel),
        std::ref(lower_right_parallel),
        std::ref(multi_parallel)
    );

    std::cout
        << "\n========== Lower Plate Shape Test ==========\n"
        << "0: lower straight -> initial shape\n"
        << "1: lower initial shape -> straight\n"
        << "9: quit\n"
        << "===========================================\n";

    while (g_running)
    {
        std::cout
            << "\nInput command: ";

        int input =
            -1;

        std::cin
            >> input;

        if (!std::cin)
        {
            std::cerr
                << "[Main] Invalid input stream.\n";

            break;
        }

        if (input == 0)
        {
            if (g_busy)
            {
                std::cout
                    << "[Main] Motor thread is busy.\n";

                continue;
            }

            bool accepted =
                RequestCommand(
                    SimpleCommand::FormInitialShape
                );

            if (!accepted)
            {
                std::cout
                    << "[Main] Previous command is still pending.\n";
            }
        }
        else if (input == 1)
        {
            if (g_busy)
            {
                std::cout
                    << "[Main] Motor thread is busy.\n";

                continue;
            }

            bool accepted =
                RequestCommand(
                    SimpleCommand::ResetToStraight
                );

            if (!accepted)
            {
                std::cout
                    << "[Main] Previous command is still pending.\n";
            }
        }
        else if (input == 9)
        {
            RequestCommand(
                SimpleCommand::Stop
            );

            g_running =
                false;

            break;
        }
        else
        {
            std::cout
                << "[Main] Unknown command. Use 0, 1, or 9.\n";
        }
    }

    if (motor_thread.joinable())
    {
        motor_thread.join();
    }

    std::cout
        << "[Main] Program exit.\n";

    return 0;
}