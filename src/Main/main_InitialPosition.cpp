#include "Ethercat/config.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/pdo_config.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "time_utils.hpp"

#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"

#include "Motor/Motor_Control.hpp"
#include "Motor/Parallel_Control.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

static void WaitUserEnter(const std::string& message)
{
    std::cout << "\n==================================================\n";
    std::cout << message << "\n";
    std::cout << "完成后按 Enter 继续...\n";
    std::cout << "==================================================\n";

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

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

    MotorControl motor_control(master, axes);

    ParallelControl upper_left_parallel(motor_control,ParallelSide::UpperLeft);

    ParallelControl lower_left_parallel(motor_control,ParallelSide::LowerLeft);

    const std::string home_file_path ="home_encoder_counts.txt";

    
    return 0;

}