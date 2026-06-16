#pragma once

class ThreePRR
{
public:
    // 机构参数
    struct Param
    {
        double l1;      // 杆1长度
        double l2;      // 杆2长度
        double l3;      // 杆3长度
        double lPS;     // P点到S点的距离
        double b;       // P点到Q点的距离

        int sideY;      // 装配侧，+1 或 -1
        double phiMin;  // 最小姿态角，rad
        double phiMax;  // 最大姿态角，rad
        int thetaSign;  // 第三支链装配分支，-1、0、+1
    };

    // FK输出
    struct Pose
    {
        double xP;
        double yP;

        double xS;
        double yS;

        double xQ;
        double yQ;

        double phi;
        double theta;
    };

    // IK输入
    struct TargetPose
    {
        double xQ;
        double yQ;
        double phi;
    };

    // IK符号分支
    struct IKSign
    {
        int sign1;
        int sign2;
        int sign3;
    };

    // IK输出
    struct JointPosition
    {
        double rho1;
        double rho2;
        double rho3;
    };

public:
    // 构造函数
    ThreePRR(const Param& param, const IKSign& ik_sign);

    // 正运动学
    Pose FK(double rho1, double rho2, double rho3) const;

    // 逆运动学
    JointPosition IK(const TargetPose& target) const;

    

    // 获取参数
    const Param& GetParam() const;

private:
    Param param_;
    IKSign ik_sign_;
};

extern ThreePRR Left_Parallel;
extern ThreePRR Right_Parallel;