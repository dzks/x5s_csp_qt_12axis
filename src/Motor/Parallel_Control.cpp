#include "Motor/Parallel_Control.hpp"
#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"
#include "Motor/Motor_Control.hpp"
#include "time_utils.hpp"

#include <array>
#include <cstdint>
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

    std::cout << target_rho_mm[1];
    std::cout << target_rho_mm[2];
    std::cout << target_rho_mm[3];

    // 3. 检查 rho 是否在有效范围内
    constexpr double kRhoMinMm = 0.0;
    constexpr double kRhoMaxMm = 1290.0;

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

    // 4. 检查当前并联机构的 3 个 rho 是否从小到大
    if (!(target_rho_mm[0] < target_rho_mm[1] &&
          target_rho_mm[1] < target_rho_mm[2]))
    {
        std::cerr << "[Parallel Error] IK position failed. "
                  << "rho order is invalid for axes {"
                  << axis_indices[0] << ", "
                  << axis_indices[1] << ", "
                  << axis_indices[2] << "}.\n";

        std::cerr << "rho values: "
                  << target_rho_mm[0] << ", "
                  << target_rho_mm[1] << ", "
                  << target_rho_mm[2] << "\n";

        throw std::runtime_error("ParallelControl::BuildTargetCountsFromPose failed: rho order invalid");
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

        std::cout << "[Parallel] axis " << axis_index
                  << " target_rho = " << target_rho_mm[i]
                  << " displacement = " << displacement_mm;
    }

    return target_counts;
}









































bool ParallelControl::MoveParallelToTargetPose(
    const ThreePRR::TargetPose& target,
    int32_t base_step_counts_per_cycle,
    int timeout_cycles
)
{
    if (base_step_counts_per_cycle <= 0)
    {
        std::cerr << "[Parallel Move] base_step_counts_per_cycle must be > 0.\n";
        return false;
    }

    std::array<int, 3> axis_indices = SelectAxisIndices();

    std::array<int32_t, 3> target_counts =
        BuildTargetCountsFromPose(target);

    EthercatMaster& master = motor_control_.Master();

    std::array<int32_t, 3> start_counts{};
    std::array<int64_t, 3> move_distance_counts{};

    master.setApplicationTime(getMonotonicTimeNs());
    master.receive();
    master.checkState();

    int64_t max_abs_distance = 0;

    for (int i = 0; i < 3; ++i)
    {
        int axis_index = axis_indices[i];

        X5sAxis& axis = motor_control_.Axis(axis_index);

        axis.read();
        axis.setModeOfOperation(config::kModeCSP);

        if (!master.axisOperational(axis_index) ||
            !axis.operationEnabled())
        {
            std::cerr << "[Parallel Move] axis " << axis_index
                      << " is not Operation Enabled before move.\n";
            axis.holdCurrentPosition();
            axis.write();
            master.send();
            return false;
        }

        start_counts[i] = axis.actualPosition();

        move_distance_counts[i] =
            static_cast<int64_t>(target_counts[i])
            - static_cast<int64_t>(start_counts[i]);

        int64_t abs_distance = std::llabs(move_distance_counts[i]);

        if (abs_distance > max_abs_distance)
        {
            max_abs_distance = abs_distance;
        }

        std::cout << "[Parallel Move] axis " << axis_index
                  << " start = " << start_counts[i]
                  << " target = " << target_counts[i]
                  << " distance = " << move_distance_counts[i]
                  << "\n";
    }

    if (max_abs_distance == 0)
    {
        std::cout << "[Parallel Move] Already at target pose.\n";
        return true;
    }

    int64_t planned_move_cycles =
        (max_abs_distance + base_step_counts_per_cycle - 1)
        / base_step_counts_per_cycle;

    if (planned_move_cycles < 1)
    {
        planned_move_cycles = 1;
    }

    std::array<int32_t, 3> step_counts_per_cycle{};

    for (int i = 0; i < 3; ++i)
    {
        int64_t abs_distance = std::llabs(move_distance_counts[i]);

        if (abs_distance == 0)
        {
            step_counts_per_cycle[i] = 1;
        }
        else
        {
            int64_t step =
                (abs_distance + planned_move_cycles - 1)
                / planned_move_cycles;

            if (step < 1)
            {
                step = 1;
            }

            step_counts_per_cycle[i] =
                static_cast<int32_t>(step);
        }

        std::cout << "[Parallel Move] axis " << axis_indices[i]
                  << " step = " << step_counts_per_cycle[i]
                  << " counts/cycle"
                  <<"\n";
    }

    for (int i = 0; i < 3; ++i)
    {
        int axis_index = axis_indices[i];

        motor_control_.Axis(axis_index).startCspMoveTo(
            target_counts[i]
        );
    }

    auto next_time = std::chrono::steady_clock::now();

    for (int cycle = 0; cycle < timeout_cycles; ++cycle)
    {
        next_time += std::chrono::nanoseconds(config::kCycleTimeNs);

        master.setApplicationTime(getMonotonicTimeNs());
        master.receive();
        master.checkState();

        bool all_done = true;

        for (int i = 0; i < 3; ++i)
        {
            int axis_index = axis_indices[i];

            X5sAxis& axis = motor_control_.Axis(axis_index);

            axis.read();
            axis.setModeOfOperation(config::kModeCSP);

            if (!master.axisOperational(axis_index) ||
                !axis.operationEnabled())
            {
                std::cerr << "[Parallel Move] axis " << axis_index
                          << " lost Operation Enabled during move.\n";

                axis.holdCurrentPosition();
                axis.write();
                master.send();

                return false;
            }

            if (!axis.cspMoveDone())
            {
                axis.updateCspTarget(step_counts_per_cycle[i]);
                all_done = false;
            }
            else
            {
                axis.holdCurrentPosition();
            }

            axis.write();
        }

        motor_control_.SyncClocks();
        master.send();

        if (all_done)
        {
            std::cout << "[Parallel Move] Target pose reached.\n";
            return true;
        }

        std::this_thread::sleep_until(next_time);
    }

    std::cerr << "[Parallel Move] MoveParallelToTargetPose timeout.\n";
    return false;
}
