#pragma once
#include "Motor/Motor_Control.hpp"
#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"
#include "Trajectory/count_trajectory_generator.hpp"

#include <array>
#include <cstdint>
#include <vector>


enum class ParallelSide{
    UpperLeft,
    UpperRight,
    LowerLeft,
    LowerRight
};

class ParallelControl {

public:
    struct ParallelEncoder{
        int32_t axis1_encoder{0};
        int32_t axis2_encoder{0};
        int32_t axis3_encoder{0};
    };

    struct ParallelTrajectoryPlan
    {
        ParallelSide side{ParallelSide::UpperLeft};

        std::array<int, 3> axis_indices{};

        trajectory::CountTrajectory count_trajectory;

        std::array<double, 3> final_rho_mm{};
    };

    struct PlateShapePlanOptions
    {
        // p = 0 表示直板，p = 1 表示初始板型
        double p_start{0.0};
        double p_target{1.0};

        // 当前这个并联机构在直板状态下的 Q 点坐标
        double straight_xQ{0.0};
        double straight_yQ{140.0};
        double straight_phi{0.0};

        // 左端从直板到初始板型的竖直位移
        double left_shape_dy_mm{50.0};

        // 右端从直板到初始板型的竖直位移
        double right_shape_dy_mm{-35.0};

        // 右端从直板到初始板型的水平随动位移
        // 根据你的 ANSYS 数据，约为 -7.676 mm
        double right_shape_dx_mm{-7.676};

        // 如果暂时不考虑平台转角，就保持 0
        double shape_dphi_rad{0.0};
    };
public:

    ParallelControl(MotorControl& motor_control,ParallelSide side);

    //计算target pose到home的编码器变化值
    std::array<int32_t, 3> BuildTargetCountsFromPose(const ThreePRR::TargetPose& target);

    //获取当前并联机构编码器位置
    ParallelEncoder GetCurrentEncoder();
    //计算当前并联机构Q点位姿
    ThreePRR::TargetPose GetCurrentPose();

    std::array<double, 3> GetCurrentRhoMm();
    
    //单并联平台移动命令开发
    //绝对位置MOVEJ：固定步长
    bool MoveJAbsoluteFixStep(
            const ThreePRR::TargetPose& target,
            int32_t base_step_counts_per_cycle,
            int32_t position_tolerance_counts = 2000,  //位置运行容许误差
            int settle_cycles = 200             //保持最终target周期
        );
    
    //绝对位置MOVEJ：五次多项式轨迹规划移动
    bool MoveJAbsoluteQuintic(
            const ThreePRR::TargetPose& target,
            int64_t trajectory_cycles,
            int32_t position_tolerance_counts = 2000,
            int settle_cycles = 200);
    
    //相对位置MOVEJ：五次多项式轨迹规划移动
    bool MoveJRelativeQuintic(
            const ThreePRR::TargetPose& delta,
            int64_t trajectory_cycles,
            int32_t position_tolerance_counts = 2000,
            int settle_cycles = 200);

public:
    std::array<int, 3> AxisIndices() const;
    //多并联平台移动命令开发
    //生成并联平台MOVEJ五次多项式规划轨迹
    ParallelTrajectoryPlan BuildMoveJAbsoluteQuinticPlan(
            const ThreePRR::TargetPose& target,
            int64_t trajectory_cycles);
    //MOVEJ相对位移版本，输入是相对位置变化
    ParallelTrajectoryPlan BuildMoveJRelativeQuinticPlan(
            const ThreePRR::TargetPose& delta,
            int64_t trajectory_cycles);
    //MOVEL
    ParallelTrajectoryPlan BuildMoveLAbsoluteQuinticPlan(
        const ThreePRR::TargetPose& target,
        int64_t trajectory_cycles);

    ParallelTrajectoryPlan BuildMoveLRelativeQuinticPlan(
        const ThreePRR::TargetPose& delta,
        int64_t trajectory_cycles);

    //柔性板初始板型轨迹规划
    ParallelTrajectoryPlan BuildPlateShapeQuinticPlan(
        const PlateShapePlanOptions& options,
        int64_t trajectory_cycles);
        
private:
    //选择并联机构参数（左并联或右并联）
    ThreePRR& SelectMechanism();
    //选择并联机构电机轴号
    std::array<int, 3> SelectAxisIndices() const;
    //选择并联机构单位转换参数（主要区分上下，上下的运动方向不同）
    const axis_unit_converter& SelectConverter() const; 
    
    trajectory::AxisCount3 BuildTargetCountsFromJointPosition(
        const ThreePRR::JointPosition& joint_position
        );
    // 电机执行层
    using CountTrajectory = std::vector<std::array<int32_t, 3>>;
    bool ExecuteCountTrajectory(
        const CountTrajectory& trajectory,
        int32_t position_tolerance_counts,
        int settle_cycles
    );

private:
    MotorControl& motor_control_;
    ParallelSide side_;

    ParallelEncoder parallel_encoder_;
};