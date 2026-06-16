#pragma once
#include <cstdint>

class axis_unit_converter{

public:
    struct RackPinionAxisConfig{
        int Gear_teeth{18} ; //齿轮齿数
        double Gear_module{2} ; //齿轮模数
        double encoder_counts_per_motor_rev{131072.0} ; //电机编码器一圈的counts
        double Gear_ratio{16}; //减速比
        int direction{1};   //运动方向
    };
    axis_unit_converter(const RackPinionAxisConfig& rackpinion);
    int64_t DisplacementMmToCountDelta(double displacement) const;

private:
    RackPinionAxisConfig rackpinion_;
};

extern axis_unit_converter upper_converter;
extern axis_unit_converter lower_converter;