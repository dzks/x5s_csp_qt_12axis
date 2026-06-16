#include "Kinematics/parallel_kinematics.hpp"
#include "Kinematics/axis_unit_converter.hpp"
#include <iostream>
#include <cstdint>

int main(int argc, char* argv[]){

    constexpr double PI = 3.14159265358979323846;
    
    double rho1 =0;
    double rho2 =60;
    double rho3 =120;
    ThreePRR::Pose pose = Left_Parallel.FK(rho1,rho2,rho3);
    std::cout << "P = (" << pose.xP << ", " << pose.yP << ") mm" << "\n";
    std::cout << "S = (" << pose.xS << ", " << pose.yS << ") mm" << "\n";
    std::cout << "Q = (" << pose.xQ << ", " << pose.yQ << ") mm" << "\n";

    std::cout << "phi = "<< pose.phi * 180.0 / 3.14159265358979323846<< " deg" << "\n";

    ThreePRR::TargetPose target;
    target.xQ = 450;
    target.yQ = 140;
    target.phi = 0;
    ThreePRR::JointPosition jointPosition = Left_Parallel.IK(target);
    std::cout << "rho1 = (" << jointPosition.rho1  << ") mm" << "\n";
    std::cout << "rho2 = (" << jointPosition.rho2  << ") mm" << "\n";
    std::cout << "rho3 = (" << jointPosition.rho3  << ") mm" << "\n";   

    double displacement_mm = 10;
    int64_t upper_counts = upper_converter.DisplacementMmToCountDelta(displacement_mm);
    std::cout << "upper_counts = " << upper_counts  << "个编码器变化量" << "\n";  
    int64_t lower_counts = lower_converter.DisplacementMmToCountDelta(displacement_mm);
    std::cout << "lower_counts = " << lower_counts  << "个编码器变化量" << "\n";      

    return 0;
}