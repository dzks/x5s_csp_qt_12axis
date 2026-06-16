#include "Kinematics/parallel_kinematics.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>
#include <algorithm>

namespace
{
    constexpr double PI = 3.14159265358979323846;

    // 2D叉乘
    double cross2(double ax, double ay, double bx, double by)
    {
        return ax * by - ay * bx;
    }

    // 2D点乘
    double dot2(double ax, double ay, double bx, double by)
    {
        return ax * bx + ay * by;
    }

    // wrapToPi
    double wrapToPi(double a)
    {
        a = std::fmod(a + PI, 2.0 * PI);

        if (a < 0.0)
        {
            a += 2.0 * PI;
        }

        return a - PI;
    }

    double Deg2Rad(double deg)
    {
        return deg * PI / 180.0;
    }

    // 左并联机构参数
    ThreePRR::Param Left_Param{
        365.0,          // l1
        365.0,          // l2
        345.0,          // l3
        150.0,          // lPS
        75.0,           // b
        1,              // sideY
        Deg2Rad(-20.0), // phiMin
        Deg2Rad(20.0),  // phiMax
        -1              // thetaSign
    };

    // 左并联机构 IK 分支
    ThreePRR::IKSign Left_IK_Sign{
        -1,             // sign1
        1,              // sign2
        1               // sign3
    };

    // 右并联机构参数
    ThreePRR::Param Right_Param{
        150.0,          // l1
        150.0,          // l2
        150.0,          // l3
        100.0,          // lPS
        50.0,           // b
        1,              // sideY
        Deg2Rad(-40.0), // phiMin
        Deg2Rad(40.0),  // phiMax
        -1              // thetaSign
    };

    // 右并联机构 IK 分支
    ThreePRR::IKSign Right_IK_Sign{
        -1,             // sign1
        1,              // sign2
        1               // sign3
    };
}

// 左右并联机构对象
ThreePRR Left_Parallel(Left_Param, Left_IK_Sign);
ThreePRR Right_Parallel(Right_Param, Right_IK_Sign);

// 构造函数
ThreePRR::ThreePRR(const Param& param,const IKSign& ik_sign): param_(param),ik_sign_(ik_sign){}

// 正运动学
ThreePRR::Pose ThreePRR::FK(double rho1, double rho2, double rho3) const
{
    Pose pose;

    if (rho2 <= rho1 || rho3 <= rho2)
    {
        throw std::runtime_error("FK error: rho1, rho2, rho3 must satisfy rho1 < rho2 < rho3.");
    }

    // 1. 求 P 点坐标
    double d = rho2 - rho1;

    double xP = (rho2 * rho2- rho1 * rho1+ param_.l1 * param_.l1- param_.l2 * param_.l2)/ (2.0 * d);

    double yP_square = param_.l1 * param_.l1- (xP - rho1) * (xP - rho1);

    if (yP_square < -1e-9)
    {
        throw std::runtime_error("FK error: P point has no real solution.");
    }

    double yP = param_.sideY * std::sqrt(std::max(0.0, yP_square));

    // 2. 求 phi
    double A = 2.0 * yP * param_.lPS;
    double B = 2.0 * (xP - rho3) * param_.lPS;

    double C =param_.l3 * param_.l3 - (xP - rho3) * (xP - rho3) - yP * yP - param_.lPS * param_.lPS;

    double R = std::hypot(A, B); // R = sqrt(A^2 + B^2)

    if (R < 1e-12)
    {
        throw std::runtime_error("FK error: third-chain equation is singular.");
    }

    if (std::abs(C) > R + 1e-9)
    {
        throw std::runtime_error("FK error: third chain has no real solution.");
    }

    double ratio = C / R;
    ratio = std::clamp(ratio, -1.0, 1.0);

    double alpha = std::atan2(B, A);

    double phi_candidate_1 = wrapToPi(std::asin(ratio) - alpha);
    double phi_candidate_2 = wrapToPi(PI - std::asin(ratio) - alpha);

    std::vector<Pose> validPoses;

    auto checkCandidate = [&](double phi)
    {
        double xS = xP + param_.lPS * std::cos(phi);
        double yS = yP + param_.lPS * std::sin(phi);

        // S 点也应该在指定装配侧
        if (param_.sideY * yS < -1e-9)
        {
            return;
        }

        // 姿态角范围筛选
        if (phi < param_.phiMin || phi > param_.phiMax)
        {
            return;
        }

        // 计算第三支链装配角 theta
        double ps_x = param_.lPS * std::cos(phi);
        double ps_y = param_.lPS * std::sin(phi);

        double s_to_a3_x = rho3 - xS;
        double s_to_a3_y = -yS;

        double theta = std::atan2(cross2(ps_x, ps_y, s_to_a3_x, s_to_a3_y),dot2(ps_x, ps_y, s_to_a3_x, s_to_a3_y));

        if (param_.thetaSign < 0 && theta >= 0.0)
        {
            return;
        }

        if (param_.thetaSign > 0 && theta <= 0.0)
        {
            return;
        }

        Pose candidate;

        candidate.xP = xP;
        candidate.yP = yP;

        candidate.xS = xS;
        candidate.yS = yS;

        candidate.xQ = xP + param_.b * std::cos(phi);
        candidate.yQ = yP + param_.b * std::sin(phi);

        candidate.phi = phi;
        candidate.theta = theta;

        validPoses.push_back(candidate);
    };

    checkCandidate(phi_candidate_1);
    checkCandidate(phi_candidate_2);

    if (validPoses.empty())
    {
        throw std::runtime_error("FK error: no valid assembly branch.");
    }

    pose = validPoses[0];

    return pose;
}

// 逆运动学
ThreePRR::JointPosition ThreePRR::IK(const TargetPose& target) const{
    // 1. 由 Q 和 phi 反求 P 点
    double xP = target.xQ - param_.b * std::cos(target.phi);
    double yP = target.yQ - param_.b * std::sin(target.phi);

    // 2. 由 P 和 phi 求 S 点
    double xS = xP + param_.lPS * std::cos(target.phi);
    double yS = yP + param_.lPS * std::sin(target.phi);

    // 3. 判断三根杆是否可达
    double r1 = param_.l1 * param_.l1 - yP * yP;
    double r2 = param_.l2 * param_.l2 - yP * yP;
    double r3 = param_.l3 * param_.l3 - yS * yS;

    if (r1 < -1e-9)
    {
        throw std::runtime_error("IK error: link 1 cannot reach target.");
    }

    if (r2 < -1e-9)
    {
        throw std::runtime_error("IK error: link 2 cannot reach target.");
    }

    if (r3 < -1e-9)
    {
        throw std::runtime_error("IK error: link 3 cannot reach target.");
    }

    // 4. 根据 IK 分支符号求 rho
    JointPosition q;

    q.rho1 = xP + ik_sign_.sign1 * std::sqrt(std::max(0.0, r1));
    q.rho2 = xP + ik_sign_.sign2 * std::sqrt(std::max(0.0, r2));
    q.rho3 = xS + ik_sign_.sign3 * std::sqrt(std::max(0.0, r3));

    return q;
}