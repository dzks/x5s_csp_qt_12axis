#pragma once

#include "QT/PlateShapeTypes.hpp"

#include "Motor/Motor_Control.hpp"
#include "Motor/Parallel_Control.hpp"
#include "Motor/Multi_Parallel_Control.hpp"

#include <QObject>
#include <QString>

#include <atomic>
#include <cstdint>
#include <vector>

class PlateShapeMotionWorker : public QObject
{
    Q_OBJECT

public:
    enum class MotionCommand
    {
        None = 0,
        FormInitialShape = 1,
        ResetToStraight = 2,
        Stop = 3
    };

public:
    PlateShapeMotionWorker(
        MotorControl& motor_control,
        ParallelControl& upper_left,
        ParallelControl& upper_right,
        ParallelControl& lower_left,
        ParallelControl& lower_right,
        MultiParallelControl& multi_parallel,
        QObject* parent = nullptr
    );

    bool RequestFormInitialShape(
        PlatformScope scope
    );

    bool RequestResetToStraight(
        PlatformScope scope
    );

    void Stop();

public slots:
    void Run();

signals:
    void StateUpdated(
        const SystemPoseState& state
    );

    void StatusTextChanged(
        const QString& text
    );

    void MotionBusyChanged(
        bool busy
    );

    void MotionFinished(
        bool success,
        const QString& message
    );

private:
    void RunHoldCycle();

    bool ExecutePlateShapeMotion(
        PlatformScope scope,
        double p_start,
        double p_target
    );

    void UpdateHoldTargetsFromActual();

    void UpdateHoldTargetsFromPlans(
        const std::vector<MultiParallelControl::ParallelTrajectoryPlan>& plans
    );

    void EmitCurrentState();

    ParallelControl::PlateShapePlanOptions MakeOptions(
        double p_start,
        double p_target,
        bool is_right_side,
        bool is_lower_side
    ) const;

private:
    MotorControl& motor_control_;

    ParallelControl& upper_left_;
    ParallelControl& upper_right_;
    ParallelControl& lower_left_;
    ParallelControl& lower_right_;

    MultiParallelControl& multi_parallel_;

    std::atomic_bool running_{true};
    std::atomic_bool motion_busy_{false};

    std::atomic<int> requested_command_{
        static_cast<int>(
            MotionCommand::None
        )
    };

    std::atomic<int> requested_scope_{
        static_cast<int>(
            PlatformScope::UpperOnly
        )
    };

    std::vector<int32_t> hold_targets_;

    int ui_emit_counter_{0};
};