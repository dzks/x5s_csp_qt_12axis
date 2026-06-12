#include "control_worker.hpp"
#include "config.hpp"
#include "shared_state.hpp"

#include <QAbstractItemView>
#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cstdint>
#include <string>
#include <thread>

namespace {

QString yesNo(bool v) { return v ? "YES" : "NO"; }

QString hexWord(uint32_t v, int width = 4) {
    return "0x" + QString::number(v, 16).rightJustified(width, '0').toUpper();
}

class MainWindow : public QWidget {
public:
    MainWindow(SharedState& shared_state, std::thread& worker_thread, QWidget* parent = nullptr)
        : QWidget(parent), shared_state_(shared_state), worker_thread_(worker_thread) {
        setWindowTitle("X5S EtherCAT CSP Multi-Axis Direct Control");
        resize(1180, 620);

        auto* main_layout = new QVBoxLayout(this);

        auto* command_box = new QGroupBox("CSP target command");
        auto* command_layout = new QGridLayout(command_box);

        axis_combo_ = new QComboBox();
        for (int axis = 0; axis < config::kActiveAxisCount; ++axis) {
            axis_combo_->addItem(
                QString("Axis %1  /  EtherCAT position %2")
                    .arg(axis)
                    .arg(config::kAxisPositions[axis]),
                axis);
        }

        target_edit_ = new QLineEdit("0");
        target_edit_->setValidator(new QIntValidator(-2147483647, 2147483647, target_edit_));
        target_edit_->setPlaceholderText("absolute target position, count");

        step_edit_ = new QLineEdit(QString::number(config::kDefaultStepCountsPerCycle));
        step_edit_->setValidator(new QIntValidator(1, 1000000, step_edit_));
        step_edit_->setPlaceholderText("count per 1 ms cycle");

        send_button_ = new QPushButton("Send target to selected axis");
        hold_button_ = new QPushButton("Hold selected axis");
        hold_all_button_ = new QPushButton("Hold all active axes");
        stop_button_ = new QPushButton("Stop control loop");

        command_layout->addWidget(new QLabel("Axis:"), 0, 0);
        command_layout->addWidget(axis_combo_, 0, 1);
        command_layout->addWidget(new QLabel("Absolute target 607A:"), 1, 0);
        command_layout->addWidget(target_edit_, 1, 1);
        command_layout->addWidget(new QLabel("Step count/cycle:"), 2, 0);
        command_layout->addWidget(step_edit_, 2, 1);
        command_layout->addWidget(send_button_, 0, 2);
        command_layout->addWidget(hold_button_, 1, 2);
        command_layout->addWidget(hold_all_button_, 2, 2);
        command_layout->addWidget(stop_button_, 3, 2);

        main_layout->addWidget(command_box);

        auto* global_box = new QGroupBox("Master / loop status");
        auto* global_form = new QFormLayout(global_box);
        init_label_ = new QLabel("-");
        slaves_label_ = new QLabel("-");
        cycle_label_ = new QLabel("-");
        message_label_ = new QLabel("-");
        message_label_->setWordWrap(true);
        global_form->addRow("Master init:", init_label_);
        global_form->addRow("Responding slaves:", slaves_label_);
        global_form->addRow("Cycle:", cycle_label_);
        global_form->addRow("Message:", message_label_);
        main_layout->addWidget(global_box);

        auto* status_box = new QGroupBox("Axis status table: active axes + reserved slots");
        auto* status_layout = new QVBoxLayout(status_box);
        table_ = new QTableWidget(config::kMaxAxisCount, 13);
        table_->setHorizontalHeaderLabels(QStringList()
            << "Slot" << "Position" << "Configured" << "Online" << "AL/OP" << "CiA402/EN"
            << "6041" << "6060" << "Actual 6064" << "Cmd 607A" << "Final" << "Remain" << "Message");
        table_->verticalHeader()->setVisible(false);
        table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setStretchLastSection(true);
        table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        table_->setAlternatingRowColors(true);
        status_layout->addWidget(table_);
        main_layout->addWidget(status_box, 1);

        auto* note = new QLabel(
            QString("说明：当前程序启用 %1 轴，代码/界面预留到 %2 轴。启动后 6060 直接写 CSP=8，不先走 PP。目标位置按绝对 count 输入；真实发送仍由 1 ms 控制线程执行，Qt 只下发命令。请确保 DC 已开启且驱动器允许 CSP。")
                .arg(config::kActiveAxisCount)
                .arg(config::kMaxAxisCount));
        note->setWordWrap(true);
        main_layout->addWidget(note);

        initializeTable();

        connect(send_button_, &QPushButton::clicked, this, [this]() { sendTarget(); });
        connect(hold_button_, &QPushButton::clicked, this, [this]() { holdSelectedAxis(); });
        connect(hold_all_button_, &QPushButton::clicked, this, [this]() { shared_state_.requestHoldAllActiveAxes(); });
        connect(stop_button_, &QPushButton::clicked, this, [this]() { close(); });

        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, [this]() { updateStatus(); });
        timer_->start(100);
    }

    ~MainWindow() override {
        stopWorker();
    }

protected:
    void closeEvent(QCloseEvent* event) override {
        stopWorker();
        event->accept();
    }

private:
    void setCell(int row, int col, const QString& text) {
        QTableWidgetItem* item = table_->item(row, col);
        if (!item) {
            item = new QTableWidgetItem();
            table_->setItem(row, col, item);
        }
        item->setText(text);
    }

    void initializeTable() {
        for (int axis = 0; axis < config::kMaxAxisCount; ++axis) {
            setCell(axis, 0, QString::number(axis));
            setCell(axis, 1, QString::number(config::kAxisPositions[axis]));
            setCell(axis, 2, axis < config::kActiveAxisCount ? "YES" : "RESERVED");
            for (int col = 3; col < table_->columnCount(); ++col) {
                setCell(axis, col, axis < config::kActiveAxisCount ? "-" : "Reserved");
            }
        }
    }

    int selectedAxis() const {
        return axis_combo_->currentData().toInt();
    }

    void sendTarget() {
        bool ok_target = false;
        bool ok_step = false;
        const int target = target_edit_->text().toInt(&ok_target);
        const int step = step_edit_->text().toInt(&ok_step);

        if (!ok_target || !ok_step || step <= 0) {
            QMessageBox::warning(this, "Invalid input", "请输入有效的整数目标位置和正数步长。" );
            return;
        }

        shared_state_.requestTarget(
            selectedAxis(),
            static_cast<int32_t>(target),
            static_cast<int32_t>(step));
    }

    void holdSelectedAxis() {
        shared_state_.requestHold(selectedAxis());
    }

    void updateStatus() {
        const SystemStatusSnapshot s = shared_state_.status();

        if (s.init_failed) {
            init_label_->setText("FAILED");
        } else {
            init_label_->setText(s.master_initialized ? "OK" : "Initializing...");
        }

        slaves_label_->setText(QString::number(s.responding_slaves) +
                               QString(" / active expected ") +
                               QString::number(config::kActiveAxisCount));
        cycle_label_->setText(QString::number(static_cast<qulonglong>(s.cycle)));
        message_label_->setText(QString::fromStdString(s.message));

        for (int axis = 0; axis < config::kMaxAxisCount; ++axis) {
            const AxisStatusSnapshot& a = s.axes[axis];
            setCell(axis, 0, QString::number(axis));
            setCell(axis, 1, QString::number(config::kAxisPositions[axis]));
            setCell(axis, 2, a.configured ? "YES" : "RESERVED");
            setCell(axis, 3, a.configured ? yesNo(a.online) : "-");
            setCell(axis, 4, a.configured ? (hexWord(a.al_state, 2) + " / OP=" + yesNo(a.al_op)) : "-");
            setCell(axis, 5, a.configured ? (QString::fromLatin1(cia402StateName(a.cia_state)) + " / EN=" + yesNo(a.operation_enabled)) : "-");
            setCell(axis, 6, a.configured ? hexWord(a.status_word, 4) : "-");
            setCell(axis, 7, a.configured ? QString::number(a.mode_of_operation) : "-");
            setCell(axis, 8, a.configured ? QString::number(a.actual_position) : "-");
            setCell(axis, 9, a.configured ? QString::number(a.target_position) : "-");
            setCell(axis, 10, a.configured ? QString::number(a.final_target_position) : "-");
            setCell(axis, 11, a.configured ? QString::number(a.remaining_distance) : "-");
            setCell(axis, 12, QString::fromStdString(a.message));
        }
    }

    void stopWorker() {
        if (stopped_) return;
        stopped_ = true;
        shared_state_.stop();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    SharedState& shared_state_;
    std::thread& worker_thread_;
    bool stopped_{false};

    QComboBox* axis_combo_{nullptr};
    QLineEdit* target_edit_{nullptr};
    QLineEdit* step_edit_{nullptr};
    QPushButton* send_button_{nullptr};
    QPushButton* hold_button_{nullptr};
    QPushButton* hold_all_button_{nullptr};
    QPushButton* stop_button_{nullptr};
    QTimer* timer_{nullptr};

    QLabel* init_label_{nullptr};
    QLabel* slaves_label_{nullptr};
    QLabel* cycle_label_{nullptr};
    QLabel* message_label_{nullptr};
    QTableWidget* table_{nullptr};
};

}  // namespace

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    SharedState shared_state;
    std::thread worker_thread([&shared_state]() {
        runControlLoop(shared_state);
    });

    MainWindow window(shared_state, worker_thread);
    window.show();

    const int rc = app.exec();
    shared_state.stop();
    if (worker_thread.joinable()) {
        worker_thread.join();
    }
    return rc;
}
