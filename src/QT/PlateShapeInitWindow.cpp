#include "QT/PlateShapeInitWindow.hpp"

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>

PlateShapeWindow::PlateShapeWindow(
    MotorControl& motor_control,
    ParallelControl& upper_left,
    ParallelControl& upper_right,
    ParallelControl& lower_left,
    ParallelControl& lower_right,
    MultiParallelControl& multi_parallel,
    QWidget* parent
)
    : QMainWindow(parent)
{
    QWidget* central_widget =
        new QWidget(this);

    QVBoxLayout* main_layout =
        new QVBoxLayout(central_widget);

    view_ =
        new PlateShapeView(central_widget);

    scope_combo_ =
        new QComboBox(central_widget);

    scope_combo_->addItem(
        "仅上侧并联平台"
    );

    scope_combo_->addItem(
        "仅下侧并联平台"
    );

    scope_combo_->addItem(
        "上下并联平台同时运动"
    );

    form_button_ =
        new QPushButton(
            "直板 -> 初始板型",
            central_widget
        );

    reset_button_ =
        new QPushButton(
            "初始板型 -> 直板",
            central_widget
        );

    status_label_ =
        new QLabel(
            "状态：初始化中...",
            central_widget
        );

    QHBoxLayout* button_layout =
        new QHBoxLayout();

    button_layout->addWidget(
        form_button_
    );

    button_layout->addWidget(
        reset_button_
    );

    main_layout->addWidget(
        view_
    );

    main_layout->addWidget(
        scope_combo_
    );

    main_layout->addLayout(
        button_layout
    );

    main_layout->addWidget(
        status_label_
    );

    setCentralWidget(
        central_widget
    );

    setWindowTitle(
        "柔性板初始板型控制"
    );

    resize(
        860,
        620
    );

    worker_thread_ =
        new QThread(this);

    worker_ =
        new PlateShapeMotionWorker(
            motor_control,
            upper_left,
            upper_right,
            lower_left,
            lower_right,
            multi_parallel
        );

    worker_->moveToThread(
        worker_thread_
    );

    connect(
        worker_thread_,
        &QThread::started,
        worker_,
        &PlateShapeMotionWorker::Run
    );

    connect(
        worker_,
        &PlateShapeMotionWorker::StateUpdated,
        view_,
        &PlateShapeView::SetState
    );

    connect(
        worker_,
        &PlateShapeMotionWorker::StatusTextChanged,
        status_label_,
        &QLabel::setText
    );

    connect(
        worker_,
        &PlateShapeMotionWorker::MotionBusyChanged,
        this,
        &PlateShapeWindow::OnMotionBusyChanged
    );

    connect(
        worker_,
        &PlateShapeMotionWorker::MotionFinished,
        this,
        &PlateShapeWindow::OnMotionFinished
    );

    connect(
        form_button_,
        &QPushButton::clicked,
        this,
        &PlateShapeWindow::OnFormInitialShapeClicked
    );

    connect(
        reset_button_,
        &QPushButton::clicked,
        this,
        &PlateShapeWindow::OnResetToStraightClicked
    );

    worker_thread_->start();
}

PlateShapeWindow::~PlateShapeWindow()
{
    if (worker_ != nullptr)
    {
        worker_->Stop();
    }

    if (worker_thread_ != nullptr)
    {
        worker_thread_->quit();
        worker_thread_->wait();
    }

    delete worker_;
    worker_ =
        nullptr;
}

void PlateShapeWindow::OnFormInitialShapeClicked()
{
    PlatformScope scope =
        CurrentScope();

    bool accepted =
        worker_->RequestFormInitialShape(
            scope
        );

    if (!accepted)
    {
        QMessageBox::warning(
            this,
            "Warning",
            "当前已有运动正在执行。"
        );
    }
}

void PlateShapeWindow::OnResetToStraightClicked()
{
    PlatformScope scope =
        CurrentScope();

    bool accepted =
        worker_->RequestResetToStraight(
            scope
        );

    if (!accepted)
    {
        QMessageBox::warning(
            this,
            "Warning",
            "当前已有运动正在执行。"
        );
    }
}

void PlateShapeWindow::OnMotionBusyChanged(
    bool busy
)
{
    SetUiBusy(
        busy
    );
}

void PlateShapeWindow::OnMotionFinished(
    bool success,
    const QString& message
)
{
    status_label_->setText(
        "状态：" + message
    );

    if (!success)
    {
        QMessageBox::warning(
            this,
            "Motion Failed",
            message
        );
    }
}

PlatformScope PlateShapeWindow::CurrentScope() const
{
    int index =
        scope_combo_->currentIndex();

    if (index == 0)
    {
        return PlatformScope::UpperOnly;
    }

    if (index == 1)
    {
        return PlatformScope::LowerOnly;
    }

    return PlatformScope::UpperAndLower;
}

void PlateShapeWindow::SetUiBusy(
    bool busy
)
{
    form_button_->setEnabled(
        !busy
    );

    reset_button_->setEnabled(
        !busy
    );

    scope_combo_->setEnabled(
        !busy
    );
}