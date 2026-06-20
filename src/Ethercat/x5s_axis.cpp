#include "Ethercat/x5s_axis.hpp"
#include "Ethercat/pdo_config.hpp"
#include "Ethercat/cia402.hpp"

#include <cstdlib>
#include <ecrt.h>

X5sAxis::X5sAxis(uint8_t* domain_pd, const X5sPdoOffset* offset, int axis_index)
    : domain_pd_(domain_pd), offset_(offset), axis_index_(axis_index) {}

void X5sAxis::read() {
    status_word_ = EC_READ_U16(domain_pd_ + offset_->status_word);
    actual_position_ = EC_READ_S32(domain_pd_ + offset_->position_actual);

    // First valid read: initialize CSP command target to the actual position to avoid a jump.
    if (!target_initialized_) {
        target_position_ = actual_position_;
        final_target_position_ = actual_position_;
        target_initialized_ = true;
        csp_move_started_ = false;
        csp_move_done_ = true;
    }
}

void X5sAxis::write() {
    // 6040 is generated from the latest 6041 statusword.
    // In CSP, we do not use PP bit4 "new set-point". 607A is updated cyclically.
    control_word_ = baseControlWordForState(status_word_);

    EC_WRITE_U16(domain_pd_ + offset_->control_word, control_word_);
    EC_WRITE_S32(domain_pd_ + offset_->target_position, target_position_);

    // Keep the original 16-bit write because your PDO mapping defines 6060 as 16 bit.
    EC_WRITE_S16(domain_pd_ + offset_->mode_of_operation, mode_of_operation_);
}

void X5sAxis::setModeOfOperation(int16_t mode) {
    mode_of_operation_ = mode;
}

void X5sAxis::holdCurrentPosition() {
    target_position_ = actual_position_;
    final_target_position_ = actual_position_;
    csp_move_started_ = false;
    csp_move_done_ = true;
}

void X5sAxis::startCspMoveTo(int32_t absolute_target_position_counts) {
    if (!target_initialized_) {
        target_position_ = actual_position_;
        target_initialized_ = true;
    }

    // If no trajectory is active, start from the actual position to avoid a sudden target jump.
    // If a trajectory is already active, continue from the current commanded target_position_.
    if (!csp_move_started_ || csp_move_done_) {
        target_position_ = actual_position_;
    }

    final_target_position_ = absolute_target_position_counts;
    csp_move_started_ = true;
    csp_move_done_ = (target_position_ == final_target_position_);
}

void X5sAxis::updateCspTarget(int32_t step_counts_per_cycle) {
    if (!csp_move_started_ || csp_move_done_) return;

    int32_t step = std::abs(step_counts_per_cycle);
    if (step <= 0) return;

    const int32_t direction = (final_target_position_ >= target_position_) ? 1 : -1;
    const int64_t next = static_cast<int64_t>(target_position_) + static_cast<int64_t>(direction) * step;

    if ((direction > 0 && next >= final_target_position_) ||
        (direction < 0 && next <= final_target_position_)) {
        target_position_ = final_target_position_;
        csp_move_done_ = true;
    } else {
        target_position_ = static_cast<int32_t>(next);
    }
}

//新增 CSP绝对位置设置接口
void X5sAxis::setCspTargetPosition(int32_t target_position){

    target_position_ = target_position;
}

void X5sAxis::stopHold() {
    holdCurrentPosition();
}

int X5sAxis::axisIndex() const { return axis_index_; }
uint16_t X5sAxis::statusWord() const { return status_word_; }
int32_t X5sAxis::actualPosition() const { return actual_position_; }
int32_t X5sAxis::targetPosition() const { return target_position_; }
int32_t X5sAxis::finalTargetPosition() const { return final_target_position_; }
int32_t X5sAxis::remainingDistance() const { return final_target_position_ - target_position_; }
int16_t X5sAxis::modeOfOperation() const { return mode_of_operation_; }
bool X5sAxis::operationEnabled() const { return isOperationEnabled(status_word_); }
bool X5sAxis::cspMoveStarted() const { return csp_move_started_; }
bool X5sAxis::cspMoveDone() const { return csp_move_done_; }
