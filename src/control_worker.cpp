#include "control_worker.hpp"
#include "config.hpp"
#include "ethercat_master.hpp"
#include "pdo_config.hpp"
#include "time_utils.hpp"
#include "x5s_axis.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

SystemStatusSnapshot makeStatus(const EthercatMaster& master,
                                const std::vector<X5sAxis>& axes,
                                uint64_t cycle,
                                const std::array<std::string, config::kMaxAxisCount>& axis_messages,
                                const std::string& message) {
    SystemStatusSnapshot status;
    status.master_initialized = true;
    status.init_failed = false;
    status.responding_slaves = master.respondingSlaves();
    status.cycle = cycle;
    status.message = message;

    for (int axis = 0; axis < config::kMaxAxisCount; ++axis) {
        status.axes[axis].configured = false;
        status.axes[axis].message = "Reserved";
    }

    for (int axis = 0; axis < static_cast<int>(axes.size()); ++axis) {
        const X5sAxis& x5s = axes[axis];
        AxisStatusSnapshot& s = status.axes[axis];
        s.configured = true;
        s.al_op = master.axisOperational(axis);
        s.online = master.axisOnline(axis);
        s.al_state = master.axisAlState(axis);
        s.operation_enabled = x5s.operationEnabled();
        s.cia_state = parseStatusWord(x5s.statusWord());
        s.status_word = x5s.statusWord();
        s.mode_of_operation = x5s.modeOfOperation();
        s.actual_position = x5s.actualPosition();
        s.target_position = x5s.targetPosition();
        s.final_target_position = x5s.finalTargetPosition();
        s.remaining_distance = x5s.remainingDistance();
        s.csp_move_started = x5s.cspMoveStarted();
        s.csp_move_done = x5s.cspMoveDone();
        s.message = axis_messages[axis];
    }

    return status;
}

}  // namespace

void runControlLoop(SharedState& shared_state) {
    SystemStatusSnapshot status;
    status.message = "Initializing EtherCAT master...";
    shared_state.updateStatus(status);

    EthercatMaster master;
    if (!master.init()) {
        status.init_failed = true;
        status.master_initialized = false;
        status.message = "EtherCAT init failed. Check /dev/EtherCAT0, master service, permissions, slave count, and positions.";
        shared_state.updateStatus(status);
        return;
    }

    std::vector<X5sAxis> axes;
    axes.reserve(config::kActiveAxisCount);
    for (int axis = 0; axis < config::kActiveAxisCount; ++axis) {
        axes.emplace_back(master.domainData(), &g_x5s_offsets[axis], axis);
    }

    std::cout << "[INFO] Start X5S Qt CSP-only multi-axis control loop. active="
              << config::kActiveAxisCount << " reserved=" << config::kMaxAxisCount << "\n";
    std::cout << "[INFO] 6060 is written as CSP=" << config::kModeCSP
              << " from the first cyclic output. DC=" << config::kEnableDcConfig << "\n";

    auto next_time = std::chrono::steady_clock::now();
    uint64_t cycle = 0;

    std::array<int, config::kMaxAxisCount> operation_enabled_cycles{};
    std::array<bool, config::kMaxAxisCount> pending_target_valid{};
    std::array<int32_t, config::kMaxAxisCount> pending_target_position{};
    std::array<int32_t, config::kMaxAxisCount> pending_step_counts_per_cycle{};
    std::array<std::string, config::kMaxAxisCount> axis_messages{};

    for (int axis = 0; axis < config::kMaxAxisCount; ++axis) {
        pending_step_counts_per_cycle[axis] = config::kDefaultStepCountsPerCycle;
        axis_messages[axis] = (axis < config::kActiveAxisCount)
            ? "Waiting for AL OP and CiA402 Operation Enabled"
            : "Reserved";
    }

    std::string system_message = "Waiting for active axes";

    while (shared_state.isRunning()) {
        next_time += std::chrono::nanoseconds(config::kCycleTimeNs);

        master.setApplicationTime(getMonotonicTimeNs());
        master.receive();
        master.checkState();

        for (auto& axis : axes) {
            axis.read();
        }

        const UiCommandSnapshot command = shared_state.consumeCommand();
        if (!command.running) {
            break;
        }

        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            X5sAxis& axis = axes[i];
            const AxisCommandSnapshot& axis_command = command.axes[i];

            if (axis_command.new_target) {
                pending_target_position[i] = axis_command.target_position;
                pending_step_counts_per_cycle[i] = std::max<int32_t>(1, std::abs(axis_command.step_counts_per_cycle));
                pending_target_valid[i] = true;
                axis_messages[i] = "New target received from Qt UI";
                system_message = "New target received";
                std::cout << "[UI] Axis " << i
                          << " new target=" << pending_target_position[i]
                          << " step=" << pending_step_counts_per_cycle[i] << " count/cycle\n";
            }

            if (axis_command.hold_request) {
                axis.stopHold();
                pending_target_valid[i] = false;
                operation_enabled_cycles[i] = 0;
                axis_messages[i] = "Hold requested from Qt UI";
                system_message = "Hold requested";
                std::cout << "[UI] Axis " << i << " hold current position requested.\n";
            }

            // CSP-only requirement: do not pass through PP before AL OP.
            // 6060 is always written as CSP=8. If AL is not OP or CiA402 is not enabled,
            // keep 607A equal to the current actual position to avoid motion.
            axis.setModeOfOperation(config::kModeCSP);

            if (!master.axisOperational(i)) {
                axis.holdCurrentPosition();
                operation_enabled_cycles[i] = 0;
                if (pending_target_valid[i]) axis_messages[i] = "Target pending; waiting for EtherCAT AL OP";
            } else if (!axis.operationEnabled()) {
                axis.holdCurrentPosition();
                operation_enabled_cycles[i] = 0;
                if (pending_target_valid[i]) axis_messages[i] = "Target pending; waiting for CiA402 Operation Enabled";
            } else {
                ++operation_enabled_cycles[i];

                if (pending_target_valid[i] &&
                    operation_enabled_cycles[i] >= config::kWaitCyclesAfterOperationEnabled) {
                    axis.startCspMoveTo(pending_target_position[i]);
                    pending_target_valid[i] = false;
                    axis_messages[i] = "CSP move started";
                    system_message = "CSP move started";
                    std::cout << "[INFO] Axis " << i
                              << " CSP move started: actual=" << axis.actualPosition()
                              << " target=" << axis.targetPosition()
                              << " final=" << axis.finalTargetPosition()
                              << " step=" << pending_step_counts_per_cycle[i] << "\n";
                }

                axis.updateCspTarget(pending_step_counts_per_cycle[i]);
                if (axis.cspMoveStarted() && axis.cspMoveDone()) {
                    axis_messages[i] = "Target reached; holding final target";
                } else if (!pending_target_valid[i] && !axis.cspMoveStarted()) {
                    axis_messages[i] = "Operation Enabled; holding current position";
                }
            }
        }

        for (auto& axis : axes) {
            axis.write();
        }

        if (config::kEnableDcConfig) master.syncClocks();
        master.send();

        if (cycle % config::kStatusPrintPeriodCycles == 0) {
            std::cout << "[STATUS] cycle=" << cycle
                      << " responding_slaves=" << master.respondingSlaves() << "\n";
            for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
                const auto cia_state = parseStatusWord(axes[i].statusWord());
                std::cout << "  axis=" << i
                          << " pos=" << config::kAxisPositions[i]
                          << " al=0x" << std::hex << master.axisAlState(i) << std::dec
                          << " al_op=" << master.axisOperational(i)
                          << " mode=" << axes[i].modeOfOperation()
                          << " state=" << cia402StateName(cia_state)
                          << " sw=0x" << std::hex << std::setw(4) << std::setfill('0')
                          << axes[i].statusWord() << std::dec << std::setfill(' ')
                          << " actual=" << axes[i].actualPosition()
                          << " target=" << axes[i].targetPosition()
                          << " final=" << axes[i].finalTargetPosition()
                          << " remain=" << axes[i].remainingDistance()
                          << " done=" << axes[i].cspMoveDone() << "\n";
            }
        }

        shared_state.updateStatus(makeStatus(master, axes, cycle, axis_messages, system_message));
        ++cycle;
        std::this_thread::sleep_until(next_time);
    }

    std::cout << "[INFO] Stopping: all active axes hold current position for 300 cycles.\n";
    for (int i = 0; i < 300; ++i) {
        master.setApplicationTime(getMonotonicTimeNs());
        master.receive();
        for (auto& axis : axes) {
            axis.read();
            axis.setModeOfOperation(config::kModeCSP);
            axis.stopHold();
            axis.write();
        }
        if (config::kEnableDcConfig) master.syncClocks();
        master.send();
        for (int axis = 0; axis < config::kActiveAxisCount; ++axis) {
            axis_messages[axis] = "Stopping; holding current position";
        }
        shared_state.updateStatus(makeStatus(master, axes, cycle + i, axis_messages, "Stopping"));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    SystemStatusSnapshot final_status = makeStatus(master, axes, cycle, axis_messages, "Control loop stopped");
    shared_state.updateStatus(final_status);
    std::cout << "[INFO] Control loop exited.\n";
}
