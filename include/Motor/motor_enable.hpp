#pragma once

#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/x5s_axis.hpp"

#include <vector>

namespace motor
{
    bool WaitAllAxesOperationEnabled(
        EthercatMaster& master,
        std::vector<X5sAxis>& axes,
        int required_ready_cycles,
        int timeout_cycles
    );

    void HoldAllAxesCurrentPosition(
        EthercatMaster& master,
        std::vector<X5sAxis>& axes,
        int hold_cycles
    );
}