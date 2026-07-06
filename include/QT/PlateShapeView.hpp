#pragma once

#include "QT/PlateShapeTypes.hpp"

#include <QWidget>
#include <QPointF>

class PlateShapeView : public QWidget
{
public:
    explicit PlateShapeView(
        QWidget* parent = nullptr
    );

    void SetState(
        const SystemPoseState& state
    );

protected:
    void paintEvent(
        QPaintEvent* event
    ) override;

private:
    QPointF MapToView(
        double x_mm,
        double y_mm
    ) const;

private:
    SystemPoseState state_;
};