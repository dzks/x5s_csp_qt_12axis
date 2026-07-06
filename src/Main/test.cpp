#include "Ethercat/config.hpp"
#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/pdo_config.hpp"
#include "Ethercat/x5s_axis.hpp"
#include "time_utils.hpp"
#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"
#include "Motor/Motor_Control.hpp"
#include "Motor/Parallel_Control.hpp"
#include "Trajectory/count_trajectory_generator.hpp"
#include "Motor/Multi_Parallel_Control.hpp" 

#include <iostream>
#include <cstdint>
#include <vector>
#include <limits>
 
namespace
{
    constexpr double kPi = 3.14159265358979323846;

    double DegToRad(double degree)
    {
        return degree * kPi / 180.0;
    }

}


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
        axes.emplace_back(master.domainData(),&g_x5s_offsets[axis],axis);
    }

    //创建所有电机控制对象
    MotorControl motor_control(master, axes);
    motor_control.WaitAllAxesOperationEnabled(100,60000);
    motor_control.HoldAllAxesCurrentPosition(1000);
    
    //创建并联机构对象
    ParallelControl upper_left_parallel(motor_control,ParallelSide::UpperLeft);
    ParallelControl upper_right_parallel(motor_control,ParallelSide::UpperRight);
    ParallelControl lower_left_parallel(motor_control,ParallelSide::LowerLeft);
    ParallelControl lower_right_parallel(motor_control,ParallelSide::LowerRight);

    MultiParallelControl multi_parallel(motor_control,upper_left_parallel,upper_right_parallel,lower_left_parallel,lower_right_parallel);
    //左上并联
    ThreePRR::TargetPose upperlefttarget;
    upperlefttarget.xQ = 450;
    upperlefttarget.yQ = 140;
    upperlefttarget.phi = DegToRad(0) ;
    //右上并联
    ThreePRR::TargetPose upperrighttarget;
    upperrighttarget.xQ = 1145;
    upperrighttarget.yQ = 140;
    upperrighttarget.phi = DegToRad(0) ;
    //左下并联
    ThreePRR::TargetPose lowerlefttarget;
    lowerlefttarget.xQ = 450;
    lowerlefttarget.yQ = 140;
    lowerlefttarget.phi = DegToRad(0) ;
    //右下并联
    ThreePRR::TargetPose lowerrighttarget;
    lowerrighttarget.xQ = 1145;
    lowerrighttarget.yQ = 140;
    lowerrighttarget.phi = DegToRad(0) ;

    const int64_t trajectory_cycles = 5000;

    ParallelControl::ParallelTrajectoryPlan upperleftplan =
            upper_left_parallel.BuildMoveLAbsoluteQuinticPlan(upperlefttarget,trajectory_cycles);
        
    ParallelControl::ParallelTrajectoryPlan upperrightplan =
            upper_right_parallel.BuildMoveLAbsoluteQuinticPlan(upperrighttarget,trajectory_cycles);

    ParallelControl::ParallelTrajectoryPlan lowerleftplan =
            lower_left_parallel.BuildMoveLAbsoluteQuinticPlan(lowerlefttarget,trajectory_cycles);

    ParallelControl::ParallelTrajectoryPlan lowerrightplan =
            lower_right_parallel.BuildMoveLAbsoluteQuinticPlan(lowerrighttarget,trajectory_cycles);

    std::vector<MultiParallelControl::ParallelTrajectoryPlan> plans;
    plans.push_back(upperleftplan);
    plans.push_back(upperrightplan);
    plans.push_back(lowerleftplan);
    plans.push_back(lowerrightplan);

    bool move_done =multi_parallel.ExecuteSynchronizedTrajectories(plans,2000,200);
    
    if (!move_done)
    {
        std::cerr
            << "[Test] MultiParallelControl move failed.\n";

        return 1;
    }
    std::cout
        << "[Test] MultiParallelControl move success.\n";

    // [新增] 运动结束后保持当前位置
    motor_control.HoldAllAxesCurrentPosition(1000);
    
    auto upperleft_encoder = upper_left_parallel.GetCurrentPose();
    auto upperright_encoder = upper_right_parallel.GetCurrentPose();
    auto lowerleft_encoder = lower_left_parallel.GetCurrentPose();
    auto lowerright_encoder = lower_right_parallel.GetCurrentPose();
    return 0;

}