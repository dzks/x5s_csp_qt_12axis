#include "Kinematics/axis_unit_converter.hpp"
#include <cmath>
#include <cstdint>

//创建对象
axis_unit_converter::RackPinionAxisConfig upper_config{
    18,
    2.0,
    131072,
    16.0,
    1
};

axis_unit_converter::RackPinionAxisConfig lower_config{
    18,
    2.0,
    131072,
    16.0,
    -1
};

axis_unit_converter upper_converter(upper_config);
axis_unit_converter lower_converter(lower_config);

//构造函数
axis_unit_converter::axis_unit_converter(const RackPinionAxisConfig& rackpinion):rackpinion_(rackpinion){}

//位移转编码器偏差
int64_t axis_unit_converter::DisplacementMmToCountDelta(double displacement_mm) const{

    constexpr double PI = 3.14159265358979323846;

    // 1. 齿轮输出轴转一圈，齿条移动的距离
    double rack_travel_per_output_rev_mm = PI * rackpinion_.Gear_module * rackpinion_.Gear_teeth;

    // 2. 电机转一圈，齿条移动的距离
    double rack_travel_per_motor_rev_mm = rack_travel_per_output_rev_mm / rackpinion_.Gear_ratio;

    // 3. 每移动 1 mm，对应多少编码器 counts
    double counts_per_mm = rackpinion_.encoder_counts_per_motor_rev / rack_travel_per_motor_rev_mm;

    // 4. 位移增量转换成编码器增量
    double count_delta = static_cast<double>(rackpinion_.direction) * displacement_mm * counts_per_mm;

    return static_cast<int64_t>(std::llround(count_delta));

}

//编码器偏差转位移