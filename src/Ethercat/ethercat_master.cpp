#include "Ethercat/ethercat_master.hpp"
#include "Ethercat/config.hpp"
#include "Ethercat/pdo_config.hpp"
#include "time_utils.hpp"

#include <iostream>

bool EthercatMaster::init() {
    master_ = ecrt_request_master(config::kMasterIndex);
    if (!master_) {
        std::cerr << "[ERROR] Failed to request EtherCAT master.\n";
        return false;
    }

    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        std::cerr << "[ERROR] Failed to create EtherCAT domain.\n";
        return false;
    }

    for (int axis = 0; axis < config::kActiveAxisCount; ++axis) {
        const uint16_t alias = config::kAxisAliases[axis];
        const uint16_t position = config::kAxisPositions[axis];

        slave_configs_[axis] = ecrt_master_slave_config(
            master_,
            alias,
            position,
            config::kVendorId,
            config::kProductCode);

        if (!slave_configs_[axis]) {
            std::cerr << "[ERROR] Failed to get slave configuration for axis " << axis
                      << " alias=" << alias
                      << " position=" << position << ".\n";
            return false;
        }

        if (ecrt_slave_config_pdos(slave_configs_[axis], EC_END, g_syncs)) {
            std::cerr << "[ERROR] Failed to configure PDOs for axis " << axis
                      << " position=" << position << ".\n";
            return false;
        }

        if (config::kEnableDcConfig) {
            ecrt_slave_config_dc(
                slave_configs_[axis],
                config::kDcAssignActivate,
                config::kCycleTimeNs,
                0,
                0,
                0);
            std::cout << "[INFO] DC config enabled for axis " << axis
                      << " position=" << position << ".\n";
        }
    }

    buildDomainRegsForActiveAxes();
    if (ecrt_domain_reg_pdo_entry_list(domain_, g_domain_regs)) {
        std::cerr << "[ERROR] Failed to register PDO entries for active axes.\n";
        return false;
    }

    ecrt_master_application_time(master_, getMonotonicTimeNs());

    if (ecrt_master_activate(master_)) {
        std::cerr << "[ERROR] Failed to activate EtherCAT master.\n";
        return false;
    }

    domain_pd_ = ecrt_domain_data(domain_);
    if (!domain_pd_) {
        std::cerr << "[ERROR] Failed to get domain data pointer.\n";
        return false;
    }

    std::cout << "[INFO] EtherCAT master initialized for "
              << config::kActiveAxisCount << " active axes. Reserved capacity="
              << config::kMaxAxisCount << ".\n";
    return true;
}

void EthercatMaster::receive() {
    ecrt_master_receive(master_);
    ecrt_domain_process(domain_);
}

void EthercatMaster::send() {
    ecrt_domain_queue(domain_);
    ecrt_master_send(master_);
}

void EthercatMaster::setApplicationTime(uint64_t ns) {
    ecrt_master_application_time(master_, ns);
}

void EthercatMaster::syncClocks() {
    ecrt_master_sync_reference_clock(master_);
    ecrt_master_sync_slave_clocks(master_);
}

uint8_t* EthercatMaster::domainData() {
    return domain_pd_;
}

bool EthercatMaster::axisOperational(int axis_index) const {
    if (axis_index < 0 || axis_index >= config::kActiveAxisCount) return false;
    return slave_config_states_[axis_index].operational;
}

bool EthercatMaster::axisOnline(int axis_index) const {
    if (axis_index < 0 || axis_index >= config::kActiveAxisCount) return false;
    return slave_config_states_[axis_index].online;
}

uint16_t EthercatMaster::axisAlState(int axis_index) const {
    if (axis_index < 0 || axis_index >= config::kActiveAxisCount) return 0;
    return slave_config_states_[axis_index].al_state;
}

int EthercatMaster::respondingSlaves() const {
    return static_cast<int>(master_state_.slaves_responding);
}

void EthercatMaster::checkState() {
    ec_master_state_t ms{};
    ecrt_master_state(master_, &ms);
    if (ms.slaves_responding != master_state_.slaves_responding ||
        ms.al_states != master_state_.al_states ||
        ms.link_up != master_state_.link_up) {
        master_state_ = ms;
        std::cout << "[MASTER] slaves=" << master_state_.slaves_responding
                  << " al_states=0x" << std::hex << master_state_.al_states
                  << std::dec << " link=" << master_state_.link_up << "\n";
    }

    ec_domain_state_t ds{};
    ecrt_domain_state(domain_, &ds);
    if (ds.working_counter != domain_state_.working_counter ||
        ds.wc_state != domain_state_.wc_state) {
        domain_state_ = ds;
        std::cout << "[DOMAIN] wc=" << domain_state_.working_counter
                  << " wc_state=" << domain_state_.wc_state << "\n";
    }

    for (int axis = 0; axis < config::kActiveAxisCount; ++axis) {
        ec_slave_config_state_t ss{};
        ecrt_slave_config_state(slave_configs_[axis], &ss);

        if (ss.al_state != slave_config_states_[axis].al_state ||
            ss.online != slave_config_states_[axis].online ||
            ss.operational != slave_config_states_[axis].operational) {
            slave_config_states_[axis] = ss;
            std::cout << "[SLAVE " << axis
                      << " pos=" << config::kAxisPositions[axis]
                      << "] al_state=0x" << std::hex << ss.al_state
                      << std::dec << " online=" << ss.online
                      << " operational=" << ss.operational << "\n";
        }
    }
}
