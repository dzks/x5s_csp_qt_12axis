#pragma once

#include "QT/PlateShapeTypes.hpp"
#include "QT/PlateShapeView.hpp"
#include "QT/PlateShapeMotionWorker.hpp"

#include "Motor/Motor_Control.hpp"
#include "Motor/Parallel_Control.hpp"
#include "Motor/Multi_Parallel_Control.hpp"

#include <QMainWindow>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QThread>

class PlateShapeWindow : public QMainWindow
{
    Q_OBJECT

public:
    PlateShapeWindow(
        MotorControl& motor_control,
        ParallelControl& upper_left,
        ParallelControl& upper_right,
        ParallelControl& lower_left,
        ParallelControl& lower_right,
        MultiParallelControl& multi_parallel,
        QWidget* parent = nullptr
    );

    ~PlateShapeWindow() override;

private slots:
    void OnFormInitialShapeClicked();

    void OnResetToStraightClicked();

    void OnMotionBusyChanged(
        bool busy
    );

    void OnMotionFinished(
        bool success,
        const QString& message
    );

private:
    PlatformScope CurrentScope() const;

    void SetUiBusy(
        bool busy
    );

private:
    PlateShapeView* view_{nullptr};

    QComboBox* scope_combo_{nullptr};
    QPushButton* form_button_{nullptr};
    QPushButton* reset_button_{nullptr};
    QLabel* status_label_{nullptr};

    QThread* worker_thread_{nullptr};
    PlateShapeMotionWorker* worker_{nullptr};
};