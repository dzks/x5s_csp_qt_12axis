#pragma once
#include "Motor/Motor_Control.hpp"
#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"

#include <array>
#include <cstdint>
#include <vector>


enum class ParallelSide{
    UpperLeft,
    UpperRight,
    LowerLeft,
    LowerRight
};

class ParallelControl {

public:
    struct ParallelEncoder{
        int32_t axis1_encoder{0};
        int32_t axis2_encoder{0};
        int32_t axis3_encoder{0};
    };

public:

    ParallelControl(MotorControl& motor_control,ParallelSide side);

    //计算target pose到home的编码器变化值
    std::array<int32_t, 3> BuildTargetCountsFromPose(const ThreePRR::TargetPose& target);

    //获取当前并联机构编码器位置
    ParallelEncoder GetCurrentEncoder();
    //计算当前并联机构Q点位姿
    ThreePRR::TargetPose GetCurrentPose();
    
    //并联平台移动命令开发
    //固定步长
    bool MoveJAbsolute(
            const ThreePRR::TargetPose& target,
            int32_t base_step_counts_per_cycle,
            int timeout_cycles,
            int32_t position_tolerance_counts = 2000,
            int settle_cycles = 200
        );

private:
    //选择并联机构参数（左并联或右并联）
    ThreePRR& SelectMechanism();
    //选择并联机构电机轴号
    std::array<int, 3> SelectAxisIndices() const;
    //选择并联机构单位转换参数（主要区分上下，上下的运动方向不同）
    const axis_unit_converter& SelectConverter() const; 

    // 电机执行层
    using CountTrajectory = std::vector<std::array<int32_t, 3>>;
    bool ExecuteCountTrajectory(
        const CountTrajectory& trajectory,
        int32_t position_tolerance_counts,
        int settle_cycles
    );

private:
    MotorControl& motor_control_;
    ParallelSide side_;

    ParallelEncoder parallel_encoder_;
};