#include "Motor/Motor_Control.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "Ethercat/config.hpp"
#include "time_utils.hpp"
#include <vector>
#include <chrono>
#include <iostream>
#include <thread>
#include <fstream>
#include <iomanip>
#include <cstdlib>

MotorControl::MotorControl(EthercatMaster& master,std::vector<X5sAxis>& axes):master_(master),axes_(axes){}

//基础函数
//读取所有轴
void MotorControl::ReadAllAxes(){
    for (auto& axis : axes_)
    {
        axis.read();
    }
}
//写入所有轴
void MotorControl::WriteAllAxes(){
    for (auto& axis : axes_)
    {
        axis.write();
    }
}
//同步时钟
void MotorControl::SyncClocks(){
    if (config::kEnableDcConfig)
    {
        master_.syncClocks();
    }
}

X5sAxis& MotorControl::Axis(int axis_index)
{
    return axes_[axis_index];
}

EthercatMaster& MotorControl::Master()
{
    return master_;
}

int MotorControl::AxisCount() const
{
    return static_cast<int>(axes_.size());
}

// 等待使能所有轴
bool MotorControl::WaitAllAxesOperationEnabled(int required_ready_cycles,int timeout_cycles){
    std::vector<int> ready_cycles(axes_.size(), 0);
    auto next_time = std::chrono::steady_clock::now();

    for (int cycle = 0; cycle < timeout_cycles; ++cycle)
    {
        next_time += std::chrono::nanoseconds(config::kCycleTimeNs);
        master_.setApplicationTime(getMonotonicTimeNs());
        master_.receive();
        master_.checkState();

        ReadAllAxes();

        bool all_ready = true;

        for (int i = 0; i < static_cast<int>(axes_.size()); ++i)
        {
            X5sAxis& axis = axes_[i];

            axis.setModeOfOperation(config::kModeCSP);

            if (!master_.axisOperational(i))
            {
                axis.holdCurrentPosition();
                ready_cycles[i] = 0;
                all_ready = false;
            }
            else if (!axis.operationEnabled())
            {
                axis.holdCurrentPosition();
                ready_cycles[i] = 0;
                all_ready = false;
            }
            else
            {
                axis.holdCurrentPosition();
                ++ready_cycles[i];

                if (ready_cycles[i] < required_ready_cycles)
                {
                    all_ready = false;
                }
            }
        }
        WriteAllAxes();
        SyncClocks();
        master_.send();

        if (all_ready)
        {
            std::cout << "[Motor] All axes Operation Enabled.\n";
            return true;
        }

        std::this_thread::sleep_until(next_time);
    }
    std::cerr << "[Motor] WaitAllAxesOperationEnabled timeout.\n";
    return false;
}

//等待所选轴使能（用于初始home标定）
bool MotorControl::WaitAxesOperationEnabled(
    const std::vector<int>& axis_indices,
    int required_ready_cycles,
    int timeout_cycles
)
{
    std::vector<int> ready_cycles(axis_indices.size(), 0);

    auto next_time = std::chrono::steady_clock::now();

    for (int cycle = 0; cycle < timeout_cycles; ++cycle)
    {
        next_time += std::chrono::nanoseconds(config::kCycleTimeNs);

        master_.setApplicationTime(getMonotonicTimeNs());
        master_.receive();
        master_.checkState();

        for (int axis_index : axis_indices)
        {
            if (axis_index < 0 || axis_index >= static_cast<int>(axes_.size()))
            {
                std::cerr << "[Motor] Invalid axis index: "
                          << axis_index << "\n";
                return false;
            }

            axes_[axis_index].read();
        }

        bool all_ready = true;

        for (int i = 0; i < static_cast<int>(axis_indices.size()); ++i)
        {
            int axis_index = axis_indices[i];
            X5sAxis& axis = axes_[axis_index];

            axis.setModeOfOperation(config::kModeCSP);

            if (!master_.axisOperational(axis_index))
            {
                axis.holdCurrentPosition();
                ready_cycles[i] = 0;
                all_ready = false;
            }
            else if (!axis.operationEnabled())
            {
                axis.holdCurrentPosition();
                ready_cycles[i] = 0;
                all_ready = false;
            }
            else
            {
                axis.holdCurrentPosition();
                ++ready_cycles[i];

                if (ready_cycles[i] < required_ready_cycles)
                {
                    all_ready = false;
                }
            }
        }

        for (int axis_index : axis_indices)
        {
            axes_[axis_index].write();
        }

        SyncClocks();

        master_.send();

        if (all_ready)
        {
            std::cout << "[Motor] Selected axes Operation Enabled.\n";
            return true;
        }

        std::this_thread::sleep_until(next_time);
    }

    std::cerr << "[Motor] WaitAxesOperationEnabled timeout.\n";
    return false;
}


//保持所有轴的位置
void MotorControl::HoldAllAxesCurrentPosition(int hold_cycles)
{
    auto next_time = std::chrono::steady_clock::now();

    for (int cycle = 0; cycle < hold_cycles; ++cycle)
    {
        next_time += std::chrono::nanoseconds(config::kCycleTimeNs);
        master_.setApplicationTime(getMonotonicTimeNs());
        master_.receive();
        for (auto& axis : axes_)
        {
            axis.read();
            axis.setModeOfOperation(config::kModeCSP);
            axis.holdCurrentPosition();
            axis.write();
        }
        SyncClocks();
        master_.send();
        std::this_thread::sleep_until(next_time);
    }
}

//保持选定轴当前位置（用于初始home标定）
void MotorControl::HoldAxesCurrentPosition(
    const std::vector<int>& axis_indices,
    int hold_cycles
)
{
    auto next_time = std::chrono::steady_clock::now();

    for (int cycle = 0; cycle < hold_cycles; ++cycle)
    {
        next_time += std::chrono::nanoseconds(config::kCycleTimeNs);

        master_.setApplicationTime(getMonotonicTimeNs());
        master_.receive();

        for (int axis_index : axis_indices)
        {
            if (axis_index < 0 ||
                axis_index >= static_cast<int>(axes_.size()))
            {
                std::cerr << "[Motor] Invalid axis index: "
                          << axis_index << "\n";
                return;
            }

            X5sAxis& axis = axes_[axis_index];

            axis.read();
            axis.setModeOfOperation(config::kModeCSP);
            axis.holdCurrentPosition();
            axis.write();
        }

        SyncClocks();

        master_.send();

        std::this_thread::sleep_until(next_time);
    }
}


const std::array<int32_t, config::kMaxAxisCount>& MotorControl::HomeEncoderCounts() const
{
    return home_encoder_counts_;
}

const std::array<double, config::kMaxAxisCount>& MotorControl::HomeRhoMm() const
{
    return home_rho_mm_;
}

