#include "Trajectory/count_trajectory_generator.hpp"
#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <cstddef>

namespace trajectory{

namespace
{

int64_t AbsInt64(
    int64_t value
)
{
    if (value >= 0)
    {
        return value;
    }

    return -value;
}

}

//固定步长的轨迹规划（3轴同时到达）
CountTrajectory CountTrajectoryGenerator::GenerateFixedStepJoint(
    const AxisCount3& start_counts,
    const AxisCount3& target_counts,
    const FixedStepTrajectoryOptions& options
)
{
    CountTrajectory trajectory;

    if (options.base_step_counts_per_cycle <= 0)
    {
        std::cerr
            << "[Trajectory] base_step_counts_per_cycle must be > 0.\n";

        return trajectory;
    }

    std::array<int64_t, 3> delta_counts{};
    int64_t max_abs_delta = 0;

    // 1. 先计算三个轴的 delta，并找出最大位移轴
    for (int i = 0; i < 3; ++i)
    {
        delta_counts[i] =static_cast<int64_t>(target_counts[i])- static_cast<int64_t>(start_counts[i]);

        int64_t abs_delta =AbsInt64(delta_counts[i]);

        if (abs_delta > max_abs_delta)
        {
            max_abs_delta = abs_delta;
        }
    }

    // 2. 如果三个轴都不需要动
    if (max_abs_delta == 0)
    {
        trajectory.push_back(target_counts);

        return trajectory;
    }

    // 3. 计算轨迹周期数
    int64_t trajectory_cycles =
        ( max_abs_delta+ static_cast<int64_t>(options.base_step_counts_per_cycle)- 1)
        / static_cast<int64_t>(options.base_step_counts_per_cycle);

    if (trajectory_cycles < 1)
    {
        trajectory_cycles = 1;
    }

    trajectory.reserve(static_cast<std::size_t>(trajectory_cycles));

    // 4. 生成三轴同步绝对 count 轨迹
    for (int64_t cycle = 1;cycle <= trajectory_cycles;++cycle)
    {
        AxisCount3 point{};

        for (int i = 0; i < 3; ++i)
        {
            int64_t command_count =
                static_cast<int64_t>(start_counts[i])
                + delta_counts[i]
                * cycle
                / trajectory_cycles;

            point[i] =
                static_cast<int32_t>(
                    command_count
                );
        }

        trajectory.push_back(point);
    }

    // 5. 防止整数除法导致最后一点有误差
    trajectory.back() = target_counts;

    return trajectory;
}

//5次多项式轨迹规划
CountTrajectory
CountTrajectoryGenerator::GenerateQuinticJoint(
    const AxisCount3& start_counts,
    const AxisCount3& target_counts,
    const QuinticTrajectoryOptions& options
)
{
    CountTrajectory trajectory;

    if (options.trajectory_cycles <= 0)
    {
        std::cerr
            << "[Trajectory] quintic trajectory_cycles must be > 0.\n";

        return trajectory;
    }

    std::array<int64_t, 3> delta_counts{};
    int64_t max_abs_delta = 0;

    // 1. 计算三个轴的 delta，并判断是否需要运动
    for (int i = 0; i < 3; ++i)
    {
        delta_counts[i] =
            static_cast<int64_t>(target_counts[i])
            - static_cast<int64_t>(start_counts[i]);

        int64_t abs_delta =
            AbsInt64(delta_counts[i]);

        if (abs_delta > max_abs_delta)
        {
            max_abs_delta = abs_delta;
        }
    }

    // 2. 如果三个轴都不需要动，直接返回目标点
    if (max_abs_delta == 0)
    {
        trajectory.push_back(target_counts);

        return trajectory;
    }

    const int64_t trajectory_cycles = options.trajectory_cycles;

    trajectory.reserve(static_cast<std::size_t>(trajectory_cycles));

    // 3. 生成五次多项式轨迹
    for (int64_t cycle = 1;cycle <= trajectory_cycles;++cycle)
    {
        double u =static_cast<double>(cycle)/ static_cast<double>(trajectory_cycles);

        // 五次多项式归一化进度：
        // s(0)=0, s(1)=1
        // s'(0)=0, s'(1)=0
        // s''(0)=0, s''(1)=0
        double u2 = u * u;
        double u3 = u2 * u;
        double u4 = u3 * u;
        double u5 = u4 * u;

        double s =
            10.0 * u3
            - 15.0 * u4
            + 6.0 * u5;

        AxisCount3 point{};

        for (int i = 0; i < 3; ++i)
        {
            double command_count_double =
                static_cast<double>(start_counts[i])
                + static_cast<double>(delta_counts[i]) * s;

            int64_t command_count =
                static_cast<int64_t>(
                    std::llround(command_count_double)
                );

            point[i] =
                static_cast<int32_t>(
                    command_count
                );
        }

        trajectory.push_back(point);
    }

    // 4. 防止浮点误差导致最后一点不等于目标
    trajectory.back() = target_counts;

    return trajectory;
}

}

