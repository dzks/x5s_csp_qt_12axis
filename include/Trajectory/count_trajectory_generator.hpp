#pragma once
#include <cstdint>
#include <vector>
#include <array>

namespace trajectory{
    
using AxisCount3 = std::array<int32_t,3>;

using CountTrajectory = std::vector<AxisCount3>;


struct FixedStepTrajectoryOptions
{
    int32_t base_step_counts_per_cycle{500};
};


struct QuinticTrajectoryOptions
{
    // 五次多项式轨迹总周期数
    // trajectory_cycles = 1000 表示 1 秒
    int64_t trajectory_cycles{5000};
    
};

class CountTrajectoryGenerator{

public:
    //static 是指可以不创建类对象而直接调用这个函数
    //固定步长轨迹规划
    static CountTrajectory GenerateFixedStepJoint(
            const AxisCount3& start_counts,
            const AxisCount3& target_counts,
            const FixedStepTrajectoryOptions& options);

    static CountTrajectory GenerateQuinticJoint(
            const AxisCount3& start_counts,
            const AxisCount3& target_counts,
            const QuinticTrajectoryOptions& options);

private:

};

}

