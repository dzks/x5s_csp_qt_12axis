#include "Motor/Parallel_Control.hpp"
#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"
#include "Motor/Motor_Control.hpp"
#include "time_utils.hpp"
#include "Ethercat/config.hpp"
#include "Trajectory/count_trajectory_generator.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>


ParallelControl::ParallelControl(MotorControl& motor_control,ParallelSide side): motor_control_{motor_control},side_{side}{}

//选择并联机构参数（左并联或右并联）
ThreePRR& ParallelControl::SelectMechanism(){

    if (side_ == ParallelSide::UpperLeft || side_ == ParallelSide::LowerLeft)
    {
        return Left_Parallel;
    }

    return Right_Parallel;
}
//选择并联机构电机轴号
std::array<int, 3> ParallelControl::SelectAxisIndices() const
{
    switch (side_)
    {
    case ParallelSide::UpperLeft: return {0, 1, 2};
    case ParallelSide::UpperRight: return {3, 4, 5};
    case ParallelSide::LowerLeft: return {6, 7, 8};
    case ParallelSide::LowerRight: return {9, 10, 11};
    default:
        return {0, 1, 2};
    }

}
//选择并联机构单位转换参数（主要区分上下，上下的运动方向不同）
const axis_unit_converter& ParallelControl::SelectConverter() const{

    if (side_ == ParallelSide::UpperLeft || side_ == ParallelSide::UpperRight)
    {
        return upper_converter;
    }

    return lower_converter;

}

// 计算单个并联机构的电机编码器 target counts
std::array<int32_t, 3> ParallelControl::BuildTargetCountsFromPose(const ThreePRR::TargetPose& target)
{
    // 1. 选择对应的并联机构、轴号、单位转换器
    ThreePRR& mechanism = SelectMechanism();
    std::array<int, 3> axis_indices = SelectAxisIndices();
    const axis_unit_converter& converter = SelectConverter();

    // 2. 逆运动学：目标位姿 -> 三个滑块 rho
    ThreePRR::JointPosition joint_position = mechanism.IK(target);

    std::array<double, 3> target_rho_mm{
        joint_position.rho1,
        joint_position.rho2,
        joint_position.rho3
    };

    // 3. 检查 rho 是否在有效范围内
    constexpr double kRhoMinMm = 0.0;
    constexpr double kRhoMaxMm = 1360.0;

    for (int i = 0; i < 3; ++i)
    {
        int axis_index = axis_indices[i];

        if (target_rho_mm[i] < kRhoMinMm ||
            target_rho_mm[i] > kRhoMaxMm)
        {
            std::cerr << "[Parallel Error] IK position failed. ";
            throw std::runtime_error("ParallelControl::BuildTargetCountsFromPose failed: rho out of range");
        }
    }

    // 4. 检查当前并联机构的 3 个 rho 是否满足最小机械间距
    constexpr double kMinRhoGapMm = config::kMinSliderCenterGapMm;
    double gap12 = target_rho_mm[1] - target_rho_mm[0];
    double gap23 = target_rho_mm[2] - target_rho_mm[1];

    if (gap12 < kMinRhoGapMm || gap23 < kMinRhoGapMm)
    {
        std::cerr
            << "[Parallel Error] IK position failed. "
            << "rho gap is too small for axes {"
            << axis_indices[0] << ", "
            << axis_indices[1] << ", "
            << axis_indices[2] << "}.\n";

        std::cerr
            << "rho values: "
            << target_rho_mm[0] << ", "
            << target_rho_mm[1] << ", "
            << target_rho_mm[2] << "\n";

        std::cerr
            << "rho gaps: "<< "rho2-rho1 = "<< gap12<< " mm, "
            << "rho3-rho2 = "<< gap23<< " mm, "<< "required minimum = "<< kMinRhoGapMm<< " mm\n";

        throw std::runtime_error(
            "ParallelControl::BuildTargetCountsFromPose failed: rho gap too small"
        );
    }

    // 5. 获取 home 信息
    const auto& home_rho_mm = motor_control_.HomeRhoMm();
    const auto& home_encoder_counts = motor_control_.HomeEncoderCounts();

    // 6. 计算目标 encoder counts
    std::array<int32_t, 3> target_counts{};

    for (int i = 0; i < 3; ++i)
    {
        int axis_index = axis_indices[i];

        double displacement_mm = target_rho_mm[i] - home_rho_mm[axis_index];

        int64_t count_delta = converter.DisplacementMmToCountDelta(displacement_mm);

        target_counts[i] = home_encoder_counts[axis_index] + static_cast<int32_t>(count_delta);

    }

    return target_counts;
}

//获取当前某并联平台的3个编码器值
ParallelControl::ParallelEncoder ParallelControl::GetCurrentEncoder()
{
    std::array<int, 3> axis_indices = SelectAxisIndices();

    parallel_encoder_.axis1_encoder = motor_control_.GetAxisEncoderCount(axis_indices[0]);

    parallel_encoder_.axis2_encoder = motor_control_.GetAxisEncoderCount(axis_indices[1]);

    parallel_encoder_.axis3_encoder = motor_control_.GetAxisEncoderCount(axis_indices[2]);

    return parallel_encoder_;
}

//获取当前某并联平台位姿
ThreePRR::TargetPose ParallelControl::GetCurrentPose()
{
    GetCurrentEncoder();

    ThreePRR& mechanism = SelectMechanism();

    std::array<int, 3> axis_indices = SelectAxisIndices();

    const axis_unit_converter& converter = SelectConverter();

    const auto& home_encoder_counts = motor_control_.HomeEncoderCounts();

    const auto& home_rho_mm = motor_control_.HomeRhoMm();

    std::array<int32_t, 3> current_encoder_counts{
        parallel_encoder_.axis1_encoder,
        parallel_encoder_.axis2_encoder,
        parallel_encoder_.axis3_encoder
    };

    std::array<double, 3> current_rho_mm{};

    for (int i = 0; i < 3; ++i)
    {
        int axis_index = axis_indices[i];

        int64_t count_delta =
            static_cast<int64_t>(current_encoder_counts[i])
            - static_cast<int64_t>(home_encoder_counts[axis_index]);

        double displacement_mm = converter.CountDeltaToDisplacementMm(count_delta);

        current_rho_mm[i] = home_rho_mm[axis_index] + displacement_mm;

        std::cout << "[Current Pose] axis " << axis_index
                  << " rho = " << current_rho_mm[i]
                  << " mm\n";
    }

    ThreePRR::Pose fk_pose =
        mechanism.FK(
            current_rho_mm[0],
            current_rho_mm[1],
            current_rho_mm[2]
        );

    ThreePRR::TargetPose current_pose{
        fk_pose.xQ,
        fk_pose.yQ,
        fk_pose.phi
    };

    std::cout << "[Current Pose] xQ = " << current_pose.xQ
              << " mm, yQ = " << current_pose.yQ
              << " mm, phi = " << current_pose.phi
              << " rad\n";

    return current_pose;
}

//获取当前rho值
std::array<double, 3> ParallelControl::GetCurrentRhoMm()
{
    GetCurrentEncoder();

    std::array<int, 3> axis_indices =SelectAxisIndices();

    const auto& home_encoder_counts =motor_control_.HomeEncoderCounts();

    const auto& home_rho_mm =motor_control_.HomeRhoMm();

    const axis_unit_converter& converter =SelectConverter();

    std::array<double, 3> current_rho_mm{};

    std::array<int32_t, 3> actual_counts{
        parallel_encoder_.axis1_encoder,
        parallel_encoder_.axis2_encoder,
        parallel_encoder_.axis3_encoder
    };

    for (int i = 0; i < 3; ++i)
    {
        int axis_index =axis_indices[i];

        int64_t count_delta =static_cast<int64_t>(actual_counts[i])
            - static_cast<int64_t>(home_encoder_counts[axis_index]);

        double displacement_mm =converter.CountDeltaToDisplacementMm(count_delta);

        current_rho_mm[i] =home_rho_mm[axis_index]+ displacement_mm;
    }

    return current_rho_mm;
}

//MOVEJ-固定步长
bool ParallelControl::MoveJAbsoluteFixStep(
    const ThreePRR::TargetPose& target,
    int32_t base_step_counts_per_cycle,
    int32_t position_tolerance_counts,
    int settle_cycles
)
{

    // 1. 目标位姿 -> 目标绝对 encoder counts
    trajectory::AxisCount3 target_counts = BuildTargetCountsFromPose(target);

    // 2. 读取当前 encoder counts
    ParallelEncoder current_encoder = GetCurrentEncoder();

    trajectory::AxisCount3 start_counts{
        current_encoder.axis1_encoder,
        current_encoder.axis2_encoder,
        current_encoder.axis3_encoder
    };

    trajectory::FixedStepTrajectoryOptions options;
    options.base_step_counts_per_cycle = base_step_counts_per_cycle;
    trajectory::CountTrajectory trajectory =
                 trajectory::CountTrajectoryGenerator::GenerateFixedStepJoint(start_counts,target_counts,options);
    if (trajectory.empty())
    {
        std::cerr
            << "[MoveJ] Failed to generate trajectory.\n";

        return false;
    }
    // 6. 执行轨迹
    return ExecuteCountTrajectory(
        trajectory,
        position_tolerance_counts,
        settle_cycles
    );
}

//MOVEJ-五次多项式
bool ParallelControl::MoveJAbsoluteQuintic(
    const ThreePRR::TargetPose& target,
    int64_t trajectory_cycles,  //五次多项式轨迹总周期数，1000就是1s
    int32_t position_tolerance_counts,
    int settle_cycles
)
{
    if (trajectory_cycles <= 0)
    {
        std::cerr
            << "[MoveJ Quintic] trajectory_cycles must be > 0.\n";

        return false;
    }

    // 1. 目标位姿 -> 目标绝对 encoder counts
    trajectory::AxisCount3 target_counts = BuildTargetCountsFromPose(target);

    // 2. 读取当前 encoder counts
    ParallelEncoder current_encoder = GetCurrentEncoder();

    trajectory::AxisCount3 start_counts{
        current_encoder.axis1_encoder,
        current_encoder.axis2_encoder,
        current_encoder.axis3_encoder
    };

    // 3. 设置五次多项式轨迹参数
    trajectory::QuinticTrajectoryOptions options;
    options.trajectory_cycles = trajectory_cycles;

    // 4. 生成五次多项式关节空间轨迹
    trajectory::CountTrajectory trajectory =
        trajectory::CountTrajectoryGenerator::GenerateQuinticJoint(
            start_counts,
            target_counts,
            options
        );

    if (trajectory.empty())
    {
        std::cerr
            << "[MoveJ Quintic] Failed to generate trajectory.\n";

        return false;
    }

    // 5. 执行轨迹
    return ExecuteCountTrajectory(
        trajectory,
        position_tolerance_counts,
        settle_cycles
    );
}

//相对位置MOVEJ：五次多项式轨迹规划移动
bool ParallelControl::MoveJRelativeQuintic(
    const ThreePRR::TargetPose& delta,
    int64_t trajectory_cycles,
    int32_t position_tolerance_counts,
    int settle_cycles
)
{
    // 1. 获取当前末端位姿
    ThreePRR::TargetPose current_pose = GetCurrentPose();

    // 2. 当前位姿 + 相对增量 = 绝对目标位姿
    ThreePRR::TargetPose target_pose{};
    target_pose.xQ = current_pose.xQ + delta.xQ;

    target_pose.yQ = current_pose.yQ + delta.yQ;

    target_pose.phi = current_pose.phi + delta.phi;

    // 3. 调用绝对运动函数
    return MoveJAbsoluteQuintic(
        target_pose,
        trajectory_cycles,
        position_tolerance_counts,
        settle_cycles
    );
}
//生成五次多项式轨迹规划的轨迹
ParallelControl::ParallelTrajectoryPlan ParallelControl::BuildMoveJAbsoluteQuinticPlan(
    const ThreePRR::TargetPose& target,
    int64_t trajectory_cycles)
{       
    //结构体赋值
    ParallelTrajectoryPlan plan{};
    plan.side = side_;
    plan.axis_indices = AxisIndices();

     if (trajectory_cycles <= 0)
    {
        std::cerr
            << "[MoveJ Plan] trajectory_cycles must be > 0.\n";
        return plan;
    }
    ThreePRR& mechanism =SelectMechanism();

    ThreePRR::JointPosition target_joint = mechanism.IK(target);

    // 保存最终目标 rho，供 MultiParallelControl 做跨机构保护
    plan.final_rho_mm = {
        target_joint.rho1,
        target_joint.rho2,
        target_joint.rho3
    };

    trajectory::AxisCount3 target_counts = BuildTargetCountsFromPose(target);

    ParallelEncoder current_encoder = GetCurrentEncoder();

    trajectory::AxisCount3 start_counts{
        current_encoder.axis1_encoder,
        current_encoder.axis2_encoder,
        current_encoder.axis3_encoder
    };

    trajectory::QuinticTrajectoryOptions options;
    options.trajectory_cycles = trajectory_cycles;

    plan.count_trajectory =
        trajectory::CountTrajectoryGenerator::GenerateQuinticJoint(
            start_counts,
            target_counts,
            options
        );

    if (plan.count_trajectory.empty())
    {
        std::cerr
            << "[MoveJ Plan] Failed to generate quintic trajectory.\n";
    }

    return plan;

}
//生成五次多项式相对位移版本的规划轨迹
ParallelControl::ParallelTrajectoryPlan ParallelControl::BuildMoveJRelativeQuinticPlan(
    const ThreePRR::TargetPose& delta,
    int64_t trajectory_cycles)
{
    if (trajectory_cycles <= 0)
    {
        std::cerr
            << "[MoveJ Relative Plan] trajectory_cycles must be > 0.\n";

        ParallelTrajectoryPlan empty_plan{};
        empty_plan.side = side_;
        empty_plan.axis_indices = SelectAxisIndices();
        return empty_plan;
    }

    ThreePRR::TargetPose current_pose =GetCurrentPose();

    ThreePRR::TargetPose target_pose{};
    target_pose.xQ = current_pose.xQ + delta.xQ;

    target_pose.yQ = current_pose.yQ + delta.yQ;

    target_pose.phi = current_pose.phi + delta.phi;

    return BuildMoveJAbsoluteQuinticPlan(
        target_pose,
        trajectory_cycles);
}


//电机轨迹执行层（只适合单并联调试）
bool ParallelControl::ExecuteCountTrajectory(
    const CountTrajectory& trajectory,
    int32_t position_tolerance_counts,  //最终位置运行误差
    int settle_cycles   //保持target等待的周期数
)
{
    if (trajectory.empty())
    {
        std::cerr << "[Trajectory] trajectory is empty.\n";
        return false;
    }

    if (position_tolerance_counts < 0)
    {
        std::cerr << "[Trajectory] position tolerance must be >= 0.\n";
        return false;
    }

    if (settle_cycles < 0)
    {
        std::cerr << "[Trajectory] settle_cycles must be >= 0.\n";
        return false;
    }

    const std::array<int, 3> axis_indices = SelectAxisIndices();

    EthercatMaster& master = motor_control_.Master();

    auto next_time = std::chrono::steady_clock::now();

    // ---------- 轨迹执行阶段 ----------
    for (std::size_t cycle = 0;cycle < trajectory.size();++cycle)
    {
        next_time += std::chrono::nanoseconds(config::kCycleTimeNs);

        master.setApplicationTime(getMonotonicTimeNs());

        master.receive();
        master.checkState();

        bool axes_ready = true;

        for (int i = 0; i < 3; ++i)
        {
            const int axis_index = axis_indices[i];

            X5sAxis& axis = motor_control_.Axis(axis_index);

            axis.read();
            axis.setModeOfOperation(config::kModeCSP);

            if (!master.axisOperational(axis_index) || !axis.operationEnabled())
            {
                std::cerr
                    << "[Trajectory] axis "
                    << axis_index
                    << " is not Operation Enabled.\n";

                axes_ready = false;
            }
        }

        if (!axes_ready)
        {
            for (int i = 0; i < 3; ++i)
            {
                const int axis_index = axis_indices[i];

                X5sAxis& axis = motor_control_.Axis(axis_index);

                axis.holdCurrentPosition();
                axis.write();
            }

            motor_control_.SyncClocks();
            master.send();

            return false;
        }

        for (int i = 0; i < 3; ++i)
        {
            const int axis_index = axis_indices[i];
            X5sAxis& axis = motor_control_.Axis(axis_index);
            axis.setCspTargetPosition( trajectory[cycle][i] );
            axis.write();
        }

        motor_control_.SyncClocks();
        master.send();

        std::this_thread::sleep_until(
            next_time
        );
    }

    // ---------- 到终点后继续保持最终目标 ----------
    const std::array<int32_t, 3>& final_targets =
        trajectory.back();

    for (int cycle = 0;cycle < settle_cycles;++cycle)
    {
        next_time += std::chrono::nanoseconds( config::kCycleTimeNs);

        master.setApplicationTime(getMonotonicTimeNs());

        master.receive();
        master.checkState();

        bool axes_ready = true;
        bool all_reached = true;

        for (int i = 0; i < 3; ++i)
        {
            const int axis_index = axis_indices[i];

            X5sAxis& axis = motor_control_.Axis(axis_index);

            axis.read();
            axis.setModeOfOperation(config::kModeCSP);

            if (!master.axisOperational(axis_index) || !axis.operationEnabled())
            {
                std::cerr
                    << "[Trajectory] axis "
                    << axis_index
                    << " lost Operation Enabled.\n";

                axes_ready = false;
            }

            const int64_t position_error =
                std::llabs(
                    static_cast<int64_t>(
                        axis.actualPosition()
                    )
                    - static_cast<int64_t>(
                        final_targets[i]
                    )
                );

            if (position_error >
                position_tolerance_counts)
            {
                all_reached = false;
            }

            axis.setCspTargetPosition(
                final_targets[i]
            );

            axis.write();
        }

        motor_control_.SyncClocks();
        master.send();

        if (!axes_ready)
        {
            return false;
        }

        if (all_reached)
        {
            std::cout
                << "[Trajectory] final target reached.\n";

            return true;
        }

        std::this_thread::sleep_until(
            next_time
        );
    }

    std::cerr
        << "[Trajectory] final target was not reached "
        << "within settle_cycles.\n";

    return false;
}

std::array<int, 3> ParallelControl::AxisIndices() const{

    return SelectAxisIndices();
}































