#include "Ethercat/config.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/pdo_config.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "time_utils.hpp"

#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"
#include "Motor/Motor_Control.hpp"
#include "Motor/Parallel_Control.hpp"
#include <iostream>
#include <cstdint>

int main(int argc, char* argv[]){

   // 1. EtherCAT 初始化
    EthercatMaster master;

    if (!master.init())
    {
        std::cerr << "EtherCAT init failed.\n";
        return 1;
    }

    std::cout << "EtherCAT init success.\n";

    // 2. 创建轴对象
    std::vector<X5sAxis> axes;
    axes.reserve(config::kActiveAxisCount);

    for (int axis = 0; axis < config::kActiveAxisCount; ++axis)
    {
        axes.emplace_back(
            master.domainData(),
            &g_x5s_offsets[axis],
            axis
        );
    }

    //创建所有电机控制对象
    MotorControl motor_control(master, axes);
    motor_control.WaitAllAxesOperationEnabled(100,60000);
    motor_control.HoldAllAxesCurrentPosition(1000);
    
    //创建并联机构对象
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

    // auto upperleft_encoder =
    //     upper_left_parallel.GetCurrentPose();

    // auto upperright_encoder =
    //     upper_right_parallel.GetCurrentPose();

    // auto lowerleft_encoder =
    //     lower_left_parallel.GetCurrentPose();

    // auto lowerright_encoder =
    //     lower_right_parallel.GetCurrentPose();

    ThreePRR::TargetPose target;
    target.xQ = 500;
    target.yQ = 170;
    target.phi = 0 ;

    bool move_done = upper_left_parallel.MoveJAbsolute(target,200,60000);

    auto upperleft_encoder =
        upper_left_parallel.GetCurrentPose();
}