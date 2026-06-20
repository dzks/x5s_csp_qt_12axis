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
#include <chrono>       //时间工具库
#include <cstdint>      //整数类型库
#include <iostream>     //std::cout
#include <thread>       //线程
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

    // 3. 创建控制对象
    MotorControl motor_control(master, axes);

    ParallelControl upper_right_parallel(
        motor_control,
        ParallelSide::UpperRight
    );

    // 4. 只等待第一个并联机构的 3 个轴使能
    std::vector<int> upper_right_axes{
        3,4,5
    };

    bool ready = motor_control.WaitAxesOperationEnabled(
        upper_right_axes,
        config::kWaitCyclesAfterOperationEnabled,
        60000
    );

    if (!ready)
    {
        std::cerr << "[Test] UpperLeft axes enable failed.\n";
        return 1;
    }

    std::cout << "[Test] UpperLeft axes enabled.\n";

    // 5. 保持当前位置一小段时间
    motor_control.HoldAxesCurrentPosition(
        upper_right_axes,
        1000
    );

    
    ThreePRR::TargetPose safe_pose;
    safe_pose.xQ = 1145.0;
    safe_pose.yQ = 140.0;
    safe_pose.phi = 0.0;

    std::cout << "[Test] Move UpperLeft to safe pose.\n";
    std::cout << "[Test] target xQ = " << safe_pose.xQ
              << ", yQ = " << safe_pose.yQ
              << ", phi = " << safe_pose.phi
              << "\n";
    upper_right_parallel.BuildTargetCountsFromPose(safe_pose);


    // // 8. 执行第一个并联机构三轴同步运动
    // bool move_ok = upper_right_parallel.MoveParallelToTargetPose(
    //     safe_pose,
    //     500,      // base_step_counts_per_cycle，第一次建议小一点
    //     6000000
    // );

    // if (!move_ok)
    // {
    //     std::cerr << "[Test] UpperLeft move failed.\n";
    //     return 1;
    // }

    std::cout << "[Test] UpperLeft move success.\n";

    // 9. 运动结束后保持当前位置
    motor_control.HoldAxesCurrentPosition(
        upper_right_axes,
        1000
    );

    std::cout << "[Test] Finished.\n";

    return 0;

}