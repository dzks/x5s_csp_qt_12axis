#include "Ethercat/config.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/pdo_config.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "time_utils.hpp"

#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"

#include "Motor/motor_enable.hpp"
#include "Motor/motor_home.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc,char* argv[]){

    //Ethercat 初始化
    EthercatMaster master;

    if(!master.init())
    {
        std::cerr << "Ethercat init failed.\n";
        return 1;
    }
    std::cout << "EtherCAT init success.\n";

    std::vector<X5sAxis> axes;  //创建动态数组，存储对象为类：X5sAxis；因为是创建容器，所以不用传入构造函数所需参数
    axes.reserve(config::kActiveAxisCount); //预留空间，但是没有创建

    //初始化轴对象
    for (int axis = 0; axis < config::kActiveAxisCount; ++axis)
    {
        axes.emplace_back(master.domainData(), &g_x5s_offsets[axis], axis);//emplace_back = 先创建对象再push_back进去，但是这里更建议直接emplace_back，相当于在vector中直接创建对象
    }

    // 等待所有轴进入 EtherCAT OP + CiA402 Operation Enabled
    bool ready = motor::WaitAllAxesOperationEnabled(master,axes,config::kWaitCyclesAfterOperationEnabled,60000); //要求每个轴进入使能100周期才算enable成功，超过60s算超时退出

    if (!ready)
    {
        std::cerr << "Motor enable failed.\n";
        return 1;
    }

    std::cout << "All axes enabled. Holding current position.\n";

    // 只做通信测试，不运动，保持当前位置一段时间
    motor::HoldAllAxesCurrentPosition(master,axes,3000);    //让所有电机保持位置3000个控制周期

    std::cout << "Communication test finished.\n";

    return 0;
    
}