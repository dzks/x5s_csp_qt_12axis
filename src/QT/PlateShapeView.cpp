#include "QT/PlateShapeView.hpp"

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QColor>

#include <cmath>

PlateShapeView::PlateShapeView(
    QWidget* parent
)
    : QWidget(parent)
{
    setMinimumSize(
        760,
        440
    );
}

void PlateShapeView::SetState(
    const SystemPoseState& state
)
{
    state_ =
        state;

    update();
}

QPointF PlateShapeView::MapToView(
    double x_mm,
    double y_mm
) const
{
    const double margin =
        50.0;

    const double x_min =
        250.0;

    const double x_max =
        1350.0;

    const double y_min =
        60.0;

    const double y_max =
        230.0;

    const double w =
        width() - 2.0 * margin;

    const double h =
        height() - 2.0 * margin;

    double view_x =
        margin
        + (x_mm - x_min)
        / (x_max - x_min)
        * w;

    double view_y =
        height() - margin
        - (y_mm - y_min)
        / (y_max - y_min)
        * h;

    return QPointF(
        view_x,
        view_y
    );
}

void PlateShapeView::paintEvent(
    QPaintEvent* event
)
{
    (void)event;

    QPainter painter(
        this
    );

    painter.setRenderHint(
        QPainter::Antialiasing,
        true
    );

    painter.fillRect(
        rect(),
        Qt::white
    );

    painter.setPen(
        QPen(Qt::lightGray, 1)
    );

    painter.drawRect(
        20,
        20,
        width() - 40,
        height() - 40
    );

    auto drawPlatform =
        [&painter, this](
            const PlatformPose2D& pose,
            const QColor& color,
            const QString& name
        )
        {
            QPointF center =
                MapToView(
                    pose.xQ,
                    pose.yQ
                );

            painter.setPen(
                QPen(color, 2)
            );

            painter.setBrush(
                QBrush(color)
            );

            painter.drawEllipse(
                center,
                7,
                7
            );

            const double direction_length =
                32.0;

            QPointF direction_point(
                center.x()
                    + direction_length
                    * std::cos(pose.phi),
                center.y()
                    - direction_length
                    * std::sin(pose.phi)
            );

            painter.drawLine(
                center,
                direction_point
            );

            painter.setPen(
                Qt::black
            );

            painter.drawText(
                center + QPointF(10, -10),
                name
            );
        };

    QPointF upper_left_point =
        MapToView(
            state_.upper_left.xQ,
            state_.upper_left.yQ
        );

    QPointF upper_right_point =
        MapToView(
            state_.upper_right.xQ,
            state_.upper_right.yQ
        );

    QPointF lower_left_point =
        MapToView(
            state_.lower_left.xQ,
            state_.lower_left.yQ
        );

    QPointF lower_right_point =
        MapToView(
            state_.lower_right.xQ,
            state_.lower_right.yQ
        );

    painter.setPen(
        QPen(Qt::blue, 3)
    );

    painter.drawLine(
        upper_left_point,
        upper_right_point
    );

    painter.setPen(
        QPen(Qt::darkGreen, 3)
    );

    painter.drawLine(
        lower_left_point,
        lower_right_point
    );

    drawPlatform(
        state_.upper_left,
        Qt::blue,
        "UpperLeft"
    );

    drawPlatform(
        state_.upper_right,
        Qt::blue,
        "UpperRight"
    );

    drawPlatform(
        state_.lower_left,
        Qt::darkGreen,
        "LowerLeft"
    );

    drawPlatform(
        state_.lower_right,
        Qt::darkGreen,
        "LowerRight"
    );
}