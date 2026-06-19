#pragma once
#include "Motor/Motor_Control.hpp"
#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"

#include <array>
#include <cstdint>


enum class ParallelSide{
    UpperLeft,UpperRight,LowerLeft,LowerRight
};

class ParallelControl {

public:
    ParallelControl(MotorControl& motor_control,ParallelSide side);

    std::array<int32_t, 3> BuildTargetCountsFromPose(const ThreePRR::TargetPose& target);

    bool MoveParallelToTargetPose(
        const ThreePRR::TargetPose& target,
        int32_t base_step_counts_per_cycle,
        int timeout_cycles
    );


private:
    //选择并联机构参数（左并联或右并联）
    ThreePRR& SelectMechanism();
    //选择并联机构电机轴号
    std::array<int, 3> SelectAxisIndices() const;
    //选择并联机构单位转换参数（主要区分上下，上下的运动方向不同）
    const axis_unit_converter& SelectConverter() const; 

private:
    MotorControl& motor_control_;
    ParallelSide side_;

};