#include "Ethercat/config.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/pdo_config.hpp"
#include "Ethercat/x5s_axis.hpp"

#include "Motor/Motor_Control.hpp"
#include "Motor/Parallel_Control.hpp"
#include "Motor/Multi_Parallel_Control.hpp"

#include "QT/PlateShapeTypes.hpp"
#include "QT/PlateShapeInitWindow.hpp"

#include <QApplication>

#include <iostream>
#include <vector>

int main(
    int argc,
    char* argv[]
)
{
    QApplication app(
        argc,
        argv
    );

    qRegisterMetaType<SystemPoseState>(
        "SystemPoseState"
    );

    qRegisterMetaType<PlatformScope>(
        "PlatformScope"
    );

    EthercatMaster master;

    if (!master.init())
    {
        std::cerr
            << "EtherCAT init failed.\n";

        return 1;
    }

    std::cout
        << "EtherCAT init success.\n";

    std::vector<X5sAxis> axes;

    axes.reserve(
        config::kActiveAxisCount
    );

    for (int axis = 0;
         axis < config::kActiveAxisCount;
         ++axis)
    {
        axes.emplace_back(
            master.domainData(),
            &g_x5s_offsets[axis],
            axis
        );
    }

    MotorControl motor_control(
        master,
        axes
    );

    ParallelControl upper_left_parallel(
        motor_control,
        ParallelSide::UpperLeft
    );

    ParallelControl upper_right_parallel(
        motor_control,
        ParallelSide::UpperRight
    );

    ParallelControl lower_left_parallel(
        motor_control,
        ParallelSide::LowerLeft
    );

    ParallelControl lower_right_parallel(
        motor_control,
        ParallelSide::LowerRight
    );

    MultiParallelControl multi_parallel(
        motor_control,
        upper_left_parallel,
        upper_right_parallel,
        lower_left_parallel,
        lower_right_parallel
    );

    PlateShapeWindow window(
        motor_control,
        upper_left_parallel,
        upper_right_parallel,
        lower_left_parallel,
        lower_right_parallel,
        multi_parallel
    );

    window.show();

    return app.exec();
}