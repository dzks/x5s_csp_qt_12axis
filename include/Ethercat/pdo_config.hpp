#pragma once
#include "Ethercat/config.hpp"
#include <array>
#include <ecrt.h>

struct X5sPdoOffset {
    unsigned int control_word;
    unsigned int target_position;
    unsigned int mode_of_operation;
    unsigned int status_word;
    unsigned int position_actual;
};

extern std::array<X5sPdoOffset, config::kMaxAxisCount> g_x5s_offsets;
extern ec_pdo_entry_info_t g_pdo_entries[];
extern ec_pdo_info_t g_pdos[];
extern ec_sync_info_t g_syncs[];
extern ec_pdo_entry_reg_t g_domain_regs[config::kMaxAxisCount * 5 + 1];

// Rebuilds g_domain_regs for config::kActiveAxisCount axes.
// This keeps 12-axis storage reserved while only registering connected axes.
void buildDomainRegsForActiveAxes();
