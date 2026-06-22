#include "Motor/Multi_Parallel_Control.hpp"
#include "time_utils.hpp"
#include "Ethercat/config.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <algorithm>
#include <cstdlib>


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

MultiParallelControl::MultiParallelControl(
    MotorControl& motor_control,
    ParallelControl& upper_left,
    ParallelControl& upper_right,
    ParallelControl& lower_left,
    ParallelControl& lower_right)
     :motor_control_(motor_control),
      upper_left_(upper_left),
      upper_right_(upper_right),
      lower_left_(lower_left),
      lower_right_(lower_right)
{
}

//检查轨迹
bool MultiParallelControl::ValidatePlans(const std::vector<ParallelTrajectoryPlan>& plans
) const
{
    if (plans.empty())
    {
        std::cerr
            << "[MultiParallel] no trajectory plans.\n";
        return false;
    }
    std::vector<int> used_axes;
    for (const auto& plan : plans)
    {
        if (plan.count_trajectory.empty())
        {
            std::cerr
                << "[MultiParallel] empty trajectory plan.\n";
            return false;
        }
        for (int axis_index : plan.axis_indices)
        {
            if (axis_index < 0)
            {
                std::cerr
                    << "[MultiParallel] invalid axis index: "
                    << axis_index
                    << "\n";

                return false;
            }
            auto it =std::find(used_axes.begin(),used_axes.end(),axis_index);

            if (it != used_axes.end())
            {
                std::cerr
                    << "[MultiParallel] duplicate axis index: "
                    << axis_index
                    << "\n";

                return false;
            }

            used_axes.push_back(axis_index);
        }
    }

    return true;
}

//找最大轨迹长度
std::size_t MultiParallelControl::FindMaxTrajectoryCycles(
    const std::vector<ParallelTrajectoryPlan>& plans
) const
{
    std::size_t max_cycles = 0;

    for (const auto& plan : plans)
    {
        if (plan.count_trajectory.size()> max_cycles)
        {
            max_cycles = plan.count_trajectory.size();
        }
    }

    return max_cycles;
}

//收集参与运动的轴
std::vector<int> MultiParallelControl::CollectActiveAxes(
    const std::vector<ParallelTrajectoryPlan>& plans
) const
{
    std::vector<int> active_axes;

    for (const auto& plan : plans)
    {
        for (int axis_index : plan.axis_indices)
        {
            active_axes.push_back(axis_index);
        }
    }

    return active_axes;
}

//读取active axes axis中read的封装
void MultiParallelControl::ReadActiveAxes(
    const std::vector<int>& active_axes
)
{
    for (int axis_index : active_axes)
    {
        motor_control_.Axis(axis_index).read();
    }
}

//写active axes axis中write的封装
void MultiParallelControl::WriteActiveAxes(
    const std::vector<int>& active_axes
)
{
    for (int axis_index : active_axes)
    {
        motor_control_.Axis(axis_index).write();
    }
}

//设置某个周期的目标位置
void MultiParallelControl::SetTargetsForCycle(
    const std::vector<ParallelTrajectoryPlan>& plans,
    std::size_t cycle
)
{
    for (const auto& plan : plans)
    {
        std::size_t point_index = cycle;

        if (point_index >= plan.count_trajectory.size())
        {
            point_index = plan.count_trajectory.size() - 1;
        }

        const trajectory::AxisCount3& point = plan.count_trajectory[point_index];

        for (int i = 0; i < 3; ++i)
        {
            int axis_index = plan.axis_indices[i];

            motor_control_.Axis(axis_index).setCspTargetPosition(point[i]);
        }
    }
}

bool MultiParallelControl::PrepareActiveAxesForCsp(
    const std::vector<int>& active_axes
)
{
    bool axes_ready =
        true;

    EthercatMaster& master =
        motor_control_.Master();

    for (int axis_index : active_axes)
    {
        X5sAxis& axis =
            motor_control_.Axis(axis_index);

        axis.read();

        axis.setModeOfOperation(
            config::kModeCSP
        );

        bool ethercat_operational =
            master.axisOperational(axis_index);

        bool drive_enabled =
            axis.operationEnabled();

        if (!ethercat_operational
            || !drive_enabled)
        {
            std::cerr
                << "[MultiParallel] axis "
                << axis_index
                << " not ready. "
                << "EtherCAT Operational = "
                << ethercat_operational
                << ", OperationEnabled = "
                << drive_enabled
                << "\n";

            axes_ready =
                false;
        }
    }

    return axes_ready;
}

//保持轴位置不变并将不变的位置写进轴内
void MultiParallelControl::HoldActiveAxesAndWrite(
    const std::vector<int>& active_axes
)
{
    for (int axis_index : active_axes)
    {
        X5sAxis& axis = motor_control_.Axis(axis_index);

        axis.holdCurrentPosition();

        axis.write();
    }
}

void MultiParallelControl::SetFinalTargets(
    const std::vector<ParallelTrajectoryPlan>& plans
)
{
    for (const auto& plan : plans)
    {
        const trajectory::AxisCount3& final_point = plan.count_trajectory.back();

        for (int i = 0; i < 3; ++i)
        {
            int axis_index = plan.axis_indices[i];

            motor_control_.Axis(axis_index).setCspTargetPosition(final_point[i]);
        }
    }
}

//判断所有轴是否到达最终目标
bool MultiParallelControl::AreFinalTargetsReached(
    const std::vector<ParallelTrajectoryPlan>& plans,
    int32_t position_tolerance_counts
)
{
    for (const auto& plan : plans)
    {
        const trajectory::AxisCount3& final_point =
            plan.count_trajectory.back();

        for (int i = 0; i < 3; ++i)
        {
            int axis_index =plan.axis_indices[i];

            int32_t actual_position =motor_control_.Axis(axis_index).actualPosition();

            int64_t error =static_cast<int64_t>(actual_position)- static_cast<int64_t>(final_point[i]);

            if (std::llabs(error)> static_cast<int64_t>(position_tolerance_counts))
            {
                return false;
            }
        }
    }

    return true;
}
//轨迹结束后保持最终目标，并等待到位
bool MultiParallelControl::WaitForFinalTargets(
    const std::vector<ParallelTrajectoryPlan>& plans,
    const std::vector<int>& active_axes,
    int32_t position_tolerance_counts,
    int settle_cycles
)
{
    if (settle_cycles <= 0)
    {
        return AreFinalTargetsReached(plans,position_tolerance_counts);
    }

    EthercatMaster& master =motor_control_.Master();

    using Clock =std::chrono::steady_clock;

    auto next_time =Clock::now();

    for (int cycle = 0;cycle < settle_cycles;++cycle)
    {
        next_time +=std::chrono::nanoseconds(config::kCycleTimeNs);

        master.setApplicationTime(getMonotonicTimeNs());

        master.receive();
        master.checkState();

        bool axes_ready =PrepareActiveAxesForCsp(active_axes);

        if (!axes_ready)
        {
            HoldActiveAxesAndWrite(active_axes);

            motor_control_.SyncClocks();
            master.send();

            return false;
        }

        if (AreFinalTargetsReached(plans,position_tolerance_counts))
        {
            return true;
        }

        SetFinalTargets(plans);

        WriteActiveAxes(active_axes);

        motor_control_.SyncClocks();
        master.send();

        std::this_thread::sleep_until(next_time);
    }

    std::cerr
        << "[MultiParallel] final targets not reached within settle cycles.\n";

    return false;
}

//最终执行函数
bool MultiParallelControl::ExecuteSynchronizedTrajectories(
    const std::vector<ParallelTrajectoryPlan>& plans,
    int32_t position_tolerance_counts,
    int settle_cycles
)
{
    if (!ValidatePlans(plans))
    {
        return false;
    }
    
    if (!ValidateFinalRhoRelationship(plans))
    {
        return false;
    }

    if (position_tolerance_counts < 0)
    {
        std::cerr
            << "[MultiParallel] position_tolerance_counts must be >= 0.\n";

        return false;
    }

    if (settle_cycles < 0)
    {
        std::cerr
            << "[MultiParallel] settle_cycles must be >= 0.\n";

        return false;
    }

    std::size_t max_cycles =FindMaxTrajectoryCycles(plans);

    if (max_cycles == 0)
    {
        std::cerr<< "[MultiParallel] max trajectory cycles is zero.\n";

        return false;
    }

    std::vector<int> active_axes =CollectActiveAxes(plans);

    EthercatMaster& master =motor_control_.Master();

    using Clock =std::chrono::steady_clock;

    auto next_time =Clock::now();

    for (std::size_t cycle = 0;cycle < max_cycles;++cycle)
    {
        next_time +=std::chrono::nanoseconds(config::kCycleTimeNs);

        master.setApplicationTime(getMonotonicTimeNs());

        master.receive();
        master.checkState();

        bool axes_ready =PrepareActiveAxesForCsp(active_axes);

        if (!axes_ready)
        {
            HoldActiveAxesAndWrite(active_axes);

            motor_control_.SyncClocks();
            master.send();

            return false;
        }

        SetTargetsForCycle(plans,cycle);

        WriteActiveAxes(active_axes);

        motor_control_.SyncClocks();
        master.send();

        std::this_thread::sleep_until(next_time);
    }

    return WaitForFinalTargets(
        plans,
        active_axes,
        position_tolerance_counts,
        settle_cycles
    );
}

//保护函数，确认机械不会相撞
bool MultiParallelControl::ValidateFinalRhoRelationship(
    const std::vector<ParallelTrajectoryPlan>& plans
)
{
    constexpr double kMinRhoGapMm =config::kMinSliderCenterGapMm;

    std::array<double, 3> upper_left_rho =upper_left_.GetCurrentRhoMm();
    std::array<double, 3> upper_right_rho =upper_right_.GetCurrentRhoMm();
    std::array<double, 3> lower_left_rho =lower_left_.GetCurrentRhoMm();
    std::array<double, 3> lower_right_rho =lower_right_.GetCurrentRhoMm();

    for (const auto& plan : plans)
    {
        switch (plan.side)
        {
        case ParallelSide::UpperLeft:upper_left_rho =plan.final_rho_mm;break;
        case ParallelSide::UpperRight:upper_right_rho =plan.final_rho_mm;break;
        case ParallelSide::LowerLeft:lower_left_rho =plan.final_rho_mm;break;
        case ParallelSide::LowerRight:lower_right_rho =plan.final_rho_mm;break;
        }
    }
    // 上排跨机构保护：
    // axis 2 = UpperLeft rho3
    // axis 3 = UpperRight rho1
    double upper_gap =upper_right_rho[0] - upper_left_rho[2];

    if (upper_gap < kMinRhoGapMm)
    {
        std::cerr
            << "[MultiParallel] Upper row rho protection failed.\n"
            << "Expected: UpperRight rho1 - UpperLeft rho3 >= "<< kMinRhoGapMm
            << " mm\n"
            << "UpperRight rho1 = "<< upper_right_rho[0]<< " mm, UpperLeft rho3 = "
            << upper_left_rho[2]<< " mm, gap = "<< upper_gap
            << " mm\n";

        return false;
    }

    // 下排跨机构保护：
    // axis 8 = LowerLeft rho3
    // axis 9 = LowerRight rho1
    double lower_gap =
        lower_right_rho[0] - lower_left_rho[2];

    if (lower_gap < kMinRhoGapMm)
    {
        std::cerr
            << "[MultiParallel] Lower row rho protection failed.\n"
            << "Expected: LowerRight rho1 - LowerLeft rho3 >= "<< kMinRhoGapMm
            << " mm\n"
            << "LowerRight rho1 = "<< lower_right_rho[0]<< " mm, LowerLeft rho3 = "
            << lower_left_rho[2]<< " mm, gap = "<< lower_gap
            << " mm\n";

        return false;
    }

    return true;
}