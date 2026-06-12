#pragma once
#include "Ethercat/config.hpp"

#include <array>
#include <cstdint>
#include <ecrt.h>

class EthercatMaster {
public:
    bool init();
    void receive();
    void send();
    void checkState();
    void setApplicationTime(uint64_t app_time_ns);
    void syncClocks();
    uint8_t* domainData();

    bool axisOperational(int axis_index) const;
    bool axisOnline(int axis_index) const;
    uint16_t axisAlState(int axis_index) const;
    int respondingSlaves() const;

private:
    ec_master_t* master_{nullptr};
    ec_domain_t* domain_{nullptr};
    std::array<ec_slave_config_t*, config::kMaxAxisCount> slave_configs_{};
    uint8_t* domain_pd_{nullptr};

    ec_master_state_t master_state_{};
    ec_domain_state_t domain_state_{};
    std::array<ec_slave_config_state_t, config::kMaxAxisCount> slave_config_states_{};
};
