#include "Motor/motor_home.hpp"

namespace motor
{
    const std::array<int32_t, config::kActiveAxisCount> kHomeEncoderCounts{
        -740285, -2775036, -6366006,    //轴0-2
        5384489, 2514357, 2042456,      //轴3-5


    };

    const std::array<double, config::kActiveAxisCount> kHomeRhoMm{
        0.0, 60.0, 120.0,               //轴0-2
        1170.0, 1230.0, 1290.0,         //轴3-5
    };
}