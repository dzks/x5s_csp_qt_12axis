#include "Ethercat/config.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/pdo_config.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "time_utils.hpp"
#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"
#include "Motor/Motor_Control.hpp"
#include "Motor/Parallel_Control.hpp"
#include "Trajectory/count_trajectory_generator.hpp"
#include "Motor/Multi_Parallel_Control.hpp" 

#include <iostream>
#include <cstdint>
#include <vector>
#include <limits>
#include <thread>
#include <atomic>
#include <cmath>

namespace
{
    constexpr double kPi = 3.14159265358979323846;

    double DegToRad(double degree)
    {
        return degree * kPi / 180.0;
    };

    enum class SimpleCommand{
        None = -1,
        LowerForm = 0,
        LowerReset = 1,
        Stop = 9
    };

    std::atomic<int> g_command{static_cast<int>(SimpleCommand::None)};
    std::atomic_bool g_running{true};
    std::atomic_bool g_busy{false};

    enum class LowerPlateState
{
    Unknown = 0,
    Straight = 1,
    InitialShape = 2,
    Other = 3
};

const char* LowerPlateStateName(
    LowerPlateState state
)
{
    switch (state)
    {
    case LowerPlateState::Straight:
        return "Straight";

    case LowerPlateState::InitialShape:
        return "InitialShape";

    case LowerPlateState::Other:
        return "Other";

    default:
        return "Unknown";
    }
}

bool IsClose(
    double actual,
    double expected,
    double tolerance
)
{
    return std::abs(actual - expected) <= tolerance;
}

bool IsPoseClose(
    const ThreePRR::TargetPose& pose,
    double expected_xQ,
    double expected_yQ,
    double expected_phi,
    double position_tolerance_mm,
    double phi_tolerance_rad
)
{
    return IsClose(
            pose.xQ,
            expected_xQ,
            position_tolerance_mm
        )
        && IsClose(
            pose.yQ,
            expected_yQ,
            position_tolerance_mm
        )
        && IsClose(
            pose.phi,
            expected_phi,
            phi_tolerance_rad
        );
}

LowerPlateState DetectLowerPlateState(
    ParallelControl& lower_left_parallel,
    ParallelControl& lower_right_parallel
)
{
    constexpr double kPositionToleranceMm =
        2.0;

    const double kPhiToleranceRad =
        DegToRad(
            2.0
        );

    constexpr double kLeftStraightXQ =
        450.0;

    constexpr double kRightStraightXQ =
        1145.0;

    constexpr double kStraightYQ =
        140.0;

    constexpr double kLeftInitialYQ =
        190.0;

    constexpr double kRightInitialYQ =
        105.0;

    constexpr double kRightShapeDxMm =
        -7.676;

    constexpr double kStraightPhi =
        0.0;

    ThreePRR::TargetPose lower_left_pose =
        lower_left_parallel.GetCurrentPose();

    ThreePRR::TargetPose lower_right_pose =
        lower_right_parallel.GetCurrentPose();

    bool lower_left_is_straight =
        IsPoseClose(
            lower_left_pose,
            kLeftStraightXQ,
            kStraightYQ,
            kStraightPhi,
            kPositionToleranceMm,
            kPhiToleranceRad
        );

    bool lower_right_is_straight =
        IsPoseClose(
            lower_right_pose,
            kRightStraightXQ,
            kStraightYQ,
            kStraightPhi,
            kPositionToleranceMm,
            kPhiToleranceRad
        );

    if (lower_left_is_straight
        && lower_right_is_straight)
    {
        return LowerPlateState::Straight;
    }

    bool lower_left_is_initial =
        IsPoseClose(
            lower_left_pose,
            kLeftStraightXQ,
            kLeftInitialYQ,
            kStraightPhi,
            kPositionToleranceMm,
            kPhiToleranceRad
        );

    bool lower_right_is_initial =
        IsPoseClose(
            lower_right_pose,
            kRightStraightXQ + kRightShapeDxMm,
            kRightInitialYQ,
            kStraightPhi,
            kPositionToleranceMm,
            kPhiToleranceRad
        );

    if (lower_left_is_initial
        && lower_right_is_initial)
    {
        return LowerPlateState::InitialShape;
    }

    std::cerr
        << "[StateCheck] Lower plate state is neither Straight nor InitialShape.\n";

    std::cerr
        << "[StateCheck] LowerLeft pose: "
        << "xQ = "
        << lower_left_pose.xQ
        << ", yQ = "
        << lower_left_pose.yQ
        << ", phi = "
        << lower_left_pose.phi
        << "\n";

    std::cerr
        << "[StateCheck] LowerRight pose: "
        << "xQ = "
        << lower_right_pose.xQ
        << ", yQ = "
        << lower_right_pose.yQ
        << ", phi = "
        << lower_right_pose.phi
        << "\n";

    return LowerPlateState::Other;
}
}
void RunHoldCycle(
    MotorControl& motor_control,
    const std::vector<int32_t>& hold_targets
)
{
    EthercatMaster& master =motor_control.Master();

    master.setApplicationTime(getMonotonicTimeNs());
    master.receive();
    master.checkState();
    for (int axis_index = 0;axis_index < config::kActiveAxisCount;++axis_index)
    {
        X5sAxis& axis =motor_control.Axis(axis_index);
        axis.read();
        axis.setModeOfOperation(config::kModeCSP);
        axis.setCspTargetPosition(hold_targets[axis_index]);
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
    const int64_t trajectory_cycles =5000;

    ParallelControl::PlateShapePlanOptions lower_left_options;

    lower_left_options.p_start =p_start;
    lower_left_options.p_target = p_target;
    lower_left_options.straight_xQ =450.0;
    lower_left_options.straight_yQ =140.0;
    lower_left_options.straight_phi =DegToRad(0.0);
    lower_left_options.left_shape_dy_mm =50.0;
    lower_left_options.right_shape_dy_mm =-35.0;
    lower_left_options.right_shape_dx_mm =-7.676;
    lower_left_options.shape_dphi_rad =DegToRad(0.0);


    ParallelControl::PlateShapePlanOptions lower_right_options;
    lower_right_options.p_start =p_start;
    lower_right_options.p_target =p_target;
    lower_right_options.straight_xQ =1145.0;
    lower_right_options.straight_yQ =140.0;
    lower_right_options.straight_phi =DegToRad(0.0);
    lower_right_options.left_shape_dy_mm =50.0;
    lower_right_options.right_shape_dy_mm =-35.0;
    lower_right_options.right_shape_dx_mm =-7.676;
    lower_right_options.shape_dphi_rad =DegToRad(0.0);


    std::vector<MultiParallelControl::ParallelTrajectoryPlan> plans;
    plans.push_back(lower_left_parallel.BuildPlateShapeQuinticPlan(lower_left_options,trajectory_cycles));
    plans.push_back(lower_right_parallel.BuildPlateShapeQuinticPlan(lower_right_options,trajectory_cycles));

    bool ok = multi_parallel.ExecuteSynchronizedTrajectories(plans,2000,200);
    if (!ok)
    {
        std::cerr<< "[Motion] lower plate shape motion failed.\n";
        return false;
    }
    for (const auto& plan : plans)
    {
        if (plan.count_trajectory.empty())
        {
            continue;
        }
        const trajectory::AxisCount3& final_point = plan.count_trajectory.back();
        for (int i = 0;i < 3;++i)
        {
            int axis_index =plan.axis_indices[i];
            hold_targets[axis_index] =final_point[i];
        }
    }
    std::cout<< "[Motion] lower plate shape motion success.\n";
    return true;
}

void MotorThreadMain(
    MotorControl& motor_control,
    ParallelControl& lower_left_parallel,
    ParallelControl& lower_right_parallel,
    MultiParallelControl& multi_parallel
)
{
    std::cout
        << "[MotorThread] Waiting all axes Operation Enabled...\n";

    bool enabled =motor_control.WaitAllAxesOperationEnabled(100,60000);

    if (!enabled)
    {
        std::cerr<< "[MotorThread] WaitAllAxesOperationEnabled failed.\n";
        g_running =false;

        return;
    }

    std::cout<< "[MotorThread] All axes enabled.\n";

    std::vector<int32_t> hold_targets(config::kActiveAxisCount, 0);

    for (int axis_index = 0;
         axis_index < config::kActiveAxisCount;
         ++axis_index)
    {
        hold_targets[axis_index] =
            motor_control.GetAxisEncoderCount(
                axis_index
            );
    }

    std::cout
        << "[MotorThread] Initial hold targets locked.\n";

    LowerPlateState initial_state =
        DetectLowerPlateState(
            lower_left_parallel,
            lower_right_parallel
        );

    std::cout
        << "[MotorThread] Initial lower plate state = "
        << LowerPlateStateName(
            initial_state
        )
        << "\n";

    if (initial_state == LowerPlateState::Other)
    {
        std::cerr
            << "[MotorThread] Warning: lower plate is not in Straight or InitialShape.\n"
            << "[MotorThread] Command 0 and 1 will be rejected until state is valid.\n";
    }

    using Clock =
        std::chrono::steady_clock;

    auto next_time =
        Clock::now();

    while (g_running)
    {
        next_time +=
            std::chrono::nanoseconds(
                config::kCycleTimeNs
            );

        int command_value =
            g_command.exchange(
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
            g_running =
                false;

            break;
        }
        else if (command == SimpleCommand::LowerForm)
        {
            g_busy =
                true;

            std::cout
                << "[MotorThread] receive command: lower form\n";

            LowerPlateState current_state =
                DetectLowerPlateState(
                    lower_left_parallel,
                    lower_right_parallel
                );

            if (current_state != LowerPlateState::Straight)
            {
                std::cerr
                    << "[Safety] Reject lower form command.\n"
                    << "[Safety] Required state: Straight.\n"
                    << "[Safety] Current state: "
                    << LowerPlateStateName(
                        current_state
                    )
                    << "\n";

                g_busy =
                    false;

                next_time =
                    Clock::now();

                RunHoldCycle(
                    motor_control,
                    hold_targets
                );

                continue;
            }

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
                std::cout
                    << "[MotorThread] lower form failed.\n";
            }

            g_busy =
                false;

            next_time =
                Clock::now();
        }
        else if (command == SimpleCommand::LowerReset)
        {
            g_busy =
                true;

            std::cout
                << "[MotorThread] receive command: lower reset\n";

            LowerPlateState current_state =
                DetectLowerPlateState(
                    lower_left_parallel,
                    lower_right_parallel
                );

            if (current_state != LowerPlateState::InitialShape)
            {
                std::cerr
                    << "[Safety] Reject lower reset command.\n"
                    << "[Safety] Required state: InitialShape.\n"
                    << "[Safety] Current state: "
                    << LowerPlateStateName(
                        current_state
                    )
                    << "\n";

                g_busy =
                    false;

                next_time =
                    Clock::now();

                RunHoldCycle(
                    motor_control,
                    hold_targets
                );

                continue;
            }

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
                std::cout
                    << "[MotorThread] lower reset failed.\n";
            }

            g_busy =
                false;

            next_time =
                Clock::now();
        }

        RunHoldCycle(
            motor_control,
            hold_targets
        );

        std::this_thread::sleep_until(
            next_time
        );
    }

    std::cout
        << "[MotorThread] exit.\n";
}


int main(int argc, char* argv[]){

    EthercatMaster master;
    if(!master.init()){
        std::cerr<<"Ethercat init failed\n";
        return 1;
    }
    std::vector<X5sAxis> axes;
    axes.reserve(config::kActiveAxisCount);
    for(int axis = 0;axis<config::kActiveAxisCount;++axis){
        axes.emplace_back(master.domainData(),&g_x5s_offsets[axis],axis);
    }
    MotorControl motor_control(master,axes);
    ParallelControl upper_left_parallel(motor_control,ParallelSide::UpperLeft);
    ParallelControl upper_right_parallel(motor_control,ParallelSide::UpperRight);
    ParallelControl lower_left_parallel(motor_control,ParallelSide::LowerLeft);
    ParallelControl lower_right_parallel(motor_control,ParallelSide::LowerRight);
    
    MultiParallelControl multi_parallel(motor_control,upper_left_parallel,upper_right_parallel,lower_left_parallel,lower_right_parallel);
    std::thread motor_thread(
            MotorThreadMain,
            std::ref(motor_control),        //这些对象不能复制，线程里要用原对象的引用
            std::ref(lower_left_parallel),
            std::ref(lower_right_parallel),
            std::ref(multi_parallel));

    while (g_running)
    {
        std::cout<< "\n0: 下侧到初始板型\n"<< "1: 下侧复位到直板\n"<< "9: 退出\n"<< "Input: ";
        int input =-1;
        std::cin>> input;

        if (input == 0)
        {
            if (g_busy)
            {
                std::cout<< "[Main] motor busy.\n";
                continue;
            }
            g_command =static_cast<int>(SimpleCommand::LowerForm);
        }
        else if (input == 1)
        {
            if (g_busy)
            {
                std::cout<< "[Main] motor busy.\n";
                continue;
            }
            g_command =static_cast<int>(SimpleCommand::LowerReset);
        }
        else if (input == 9)
        {
            g_command =static_cast<int>(SimpleCommand::Stop);
            g_running =false;
            break;
        }
    }
    if (motor_thread.joinable())
    {
        motor_thread.join();
    }
    return 0;
}