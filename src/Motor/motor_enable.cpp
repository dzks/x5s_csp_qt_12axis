#include "Motor/motor_enable.hpp"

#include "Ethercat/config.hpp"
#include "time_utils.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

namespace motor
{
    bool WaitAllAxesOperationEnabled(
        EthercatMaster& master,
        std::vector<X5sAxis>& axes,
        int required_ready_cycles,
        int timeout_cycles
    )
    {
        std::vector<int> ready_cycles(axes.size(), 0);

        auto next_time = std::chrono::steady_clock::now();

        for (int cycle = 0; cycle < timeout_cycles; ++cycle)
        {
            next_time += std::chrono::nanoseconds(config::kCycleTimeNs);

            master.setApplicationTime(getMonotonicTimeNs());
            master.receive();
            master.checkState();

            for (auto& axis : axes)
            {
                axis.read();
            }

            bool all_ready = true;

            for (int i = 0; i < static_cast<int>(axes.size()); ++i)
            {
                X5sAxis& axis = axes[i];

                axis.setModeOfOperation(config::kModeCSP);

                if (!master.axisOperational(i))
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

            for (auto& axis : axes)
            {
                axis.write();
            }

            if (config::kEnableDcConfig)
            {
                master.syncClocks();
            }

            master.send();

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

    void HoldAllAxesCurrentPosition(
        EthercatMaster& master,
        std::vector<X5sAxis>& axes,
        int hold_cycles
    )
    {
        auto next_time = std::chrono::steady_clock::now();

        for (int cycle = 0; cycle < hold_cycles; ++cycle)
        {
            next_time += std::chrono::nanoseconds(config::kCycleTimeNs);

            master.setApplicationTime(getMonotonicTimeNs());
            master.receive();

            for (auto& axis : axes)
            {
                axis.read();
                axis.setModeOfOperation(config::kModeCSP);
                axis.holdCurrentPosition();
                axis.write();
            }

            if (config::kEnableDcConfig)
            {
                master.syncClocks();
            }

            master.send();

            std::this_thread::sleep_until(next_time);
        }
    }
}