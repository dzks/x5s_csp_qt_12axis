#pragma once
#include <cstdint>

struct X5sPdoOffset;

class X5sAxis {
public:
    X5sAxis(uint8_t* domain_pd, const X5sPdoOffset* offset, int axis_index);

    void read();
    void write();

    void setModeOfOperation(int16_t mode);
    void holdCurrentPosition();
    void startCspMoveTo(int32_t absolute_target_position_counts);
    void updateCspTarget(int32_t step_counts_per_cycle);
    void stopHold();

    int axisIndex() const;
    uint16_t statusWord() const;
    int32_t actualPosition() const;
    int32_t targetPosition() const;
    int32_t finalTargetPosition() const;
    int32_t remainingDistance() const;
    int16_t modeOfOperation() const;

    bool operationEnabled() const;
    bool cspMoveStarted() const;
    bool cspMoveDone() const;

private:
    uint8_t* domain_pd_{nullptr};
    const X5sPdoOffset* offset_{nullptr};
    int axis_index_{0};

    uint16_t status_word_{0};
    int32_t actual_position_{0};
    uint16_t control_word_{0x0006};
    int16_t mode_of_operation_{8};
    int32_t target_position_{0};
    bool target_initialized_{false};

    bool csp_move_started_{false};
    bool csp_move_done_{true};
    int32_t final_target_position_{0};
};
