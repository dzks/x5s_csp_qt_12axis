#pragma once
#include "Motor/Parallel_Control.hpp"
#include "Motor/Motor_Control.hpp"

class MultiParallelControl{

public:
    using ParallelTrajectoryPlan = ParallelControl::ParallelTrajectoryPlan;
public:
    //explicit 的作用是：禁止隐式类型转换。
    explicit MultiParallelControl(
        MotorControl& motor_control,
        ParallelControl& upper_left,
        ParallelControl& upper_right,
        ParallelControl& lower_left,
        ParallelControl& lower_right);

    //同步执行多个并联机构
    bool ExecuteSynchronizedTrajectories(
        const std::vector<ParallelTrajectoryPlan>& plans,
        int32_t position_tolerance_counts = 2000,
        int settle_cycles = 200
    );

private:
    //检查轨迹：每个plan不能为空、不能有重复 axis
    bool ValidatePlans(
        const std::vector<ParallelTrajectoryPlan>& plans
    ) const;

    //找最大轨迹长度
    std::size_t FindMaxTrajectoryCycles(
        const std::vector<ParallelTrajectoryPlan>& plans
    ) const;

    //收集参与运动的轴
    std::vector<int> CollectActiveAxes(
        const std::vector<ParallelTrajectoryPlan>& plans
    ) const;

    //读取active axes axis中read的封装
    void ReadActiveAxes(
        const std::vector<int>& active_axes
    );

    //写active axes axis中write的封装
    void WriteActiveAxes(
        const std::vector<int>& active_axes
    );

    //设置某个周期的目标位置
    void SetTargetsForCycle(
        const std::vector<ParallelTrajectoryPlan>& plans,
        std::size_t cycle
    );

    void SetFinalTargets(
        const std::vector<ParallelTrajectoryPlan>& plans
    );
    
    bool PrepareActiveAxesForCsp(
        const std::vector<int>& active_axes
    );

    void HoldActiveAxesAndWrite(
        const std::vector<int>& active_axes
    );
    //判断所有轴是否到达最终目标
    bool AreFinalTargetsReached(
        const std::vector<ParallelTrajectoryPlan>& plans,
        int32_t position_tolerance_counts
    );

    //轨迹结束后保持最终目标，并等待到位
    bool WaitForFinalTargets(
        const std::vector<ParallelTrajectoryPlan>& plans,
        const std::vector<int>& active_axes,
        int32_t position_tolerance_counts,
        int settle_cycles
    );
    //保护函数，确认机械不会相撞
    bool ValidateFinalRhoRelationship(
        const std::vector<ParallelTrajectoryPlan>& plans
    );
private:
    MotorControl& motor_control_;

    ParallelControl& upper_left_;
    ParallelControl& upper_right_;
    ParallelControl& lower_left_;
    ParallelControl& lower_right_;
};