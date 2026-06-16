#pragma once
#include<Ethercat/config.hpp>

namespace motor {

    extern const std::array<int32_t, config::kActiveAxisCount> kHomeEncoderCounts;
    extern const std::array<double, config::kActiveAxisCount> kHomeRhoMm;
    
}