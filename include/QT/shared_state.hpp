#pragma once
#include "Ethercat/cia402.hpp"
#include "Ethercat/config.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>

struct AxisCommandSnapshot {
    bool new_target{false};
    bool hold_request{false};
    int32_t target_position{0};
    int32_t step_counts_per_cycle{config::kDefaultStepCountsPerCycle};
};

struct UiCommandSnapshot {
    bool running{true};
    std::array<AxisCommandSnapshot, config::kMaxAxisCount> axes{};
};

struct AxisStatusSnapshot {
    bool configured{false};
    bool al_op{false};
    bool online{false};
    uint16_t al_state{0};
    bool operation_enabled{false};
    CiA402State cia_state{CiA402State::Unknown};
    uint16_t status_word{0};
    int16_t mode_of_operation{0};
    int32_t actual_position{0};
    int32_t target_position{0};
    int32_t final_target_position{0};
    int32_t remaining_distance{0};
    bool csp_move_started{false};
    bool csp_move_done{true};
    std::string message{"Reserved"};
};

struct SystemStatusSnapshot {
    bool master_initialized{false};
    bool init_failed{false};
    int responding_slaves{0};
    uint64_t cycle{0};
    std::string message{"Not started"};
    std::array<AxisStatusSnapshot, config::kMaxAxisCount> axes{};
};

class SharedState {
public:
    void requestTarget(int axis_index, int32_t target_position, int32_t step_counts_per_cycle) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!isValidAxis(axis_index)) return;
        auto& axis = command_.axes[axis_index];
        axis.target_position = target_position;
        axis.step_counts_per_cycle = std::max<int32_t>(1, step_counts_per_cycle);
        axis.new_target = true;
    }

    void requestHold(int axis_index) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!isValidAxis(axis_index)) return;
        command_.axes[axis_index].hold_request = true;
    }

    void requestHoldAllActiveAxes() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int axis = 0; axis < config::kActiveAxisCount; ++axis) {
            command_.axes[axis].hold_request = true;
        }
    }

    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        command_.running = false;
    }

    bool isRunning() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return command_.running;
    }

    UiCommandSnapshot consumeCommand() {
        std::lock_guard<std::mutex> lock(mutex_);
        UiCommandSnapshot copy = command_;
        for (auto& axis : command_.axes) {
            axis.new_target = false;
            axis.hold_request = false;
        }
        return copy;
    }

    void updateStatus(const SystemStatusSnapshot& status) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = status;
    }

    SystemStatusSnapshot status() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

private:
    static bool isValidAxis(int axis_index) {
        return axis_index >= 0 && axis_index < config::kActiveAxisCount;
    }

    mutable std::mutex mutex_;
    UiCommandSnapshot command_;
    SystemStatusSnapshot status_;
};
