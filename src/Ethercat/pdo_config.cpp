#include "Ethercat/pdo_config.hpp"
#include "Ethercat/config.hpp"

#include <cstddef>

std::array<X5sPdoOffset, config::kMaxAxisCount> g_x5s_offsets{};

// Every concrete PDO object used in the cyclic process data.
// Format: {object index, subindex, bit length}
ec_pdo_entry_info_t g_pdo_entries[] = {
    {0x6040, 0x00, 16}, // Controlword
    {0x607A, 0x00, 32}, // Target Position
    {0x6060, 0x00, 16}, // Modes of Operation. Kept 16-bit because the current PDO mapping uses 16 bit.
    {0x6041, 0x00, 16}, // Statusword
    {0x6064, 0x00, 32}, // Position Actual Value
};

// Group PDO entries into RxPDO and TxPDO.
ec_pdo_info_t g_pdos[] = {
    {0x1601, 3, g_pdo_entries + 0}, // RxPDO: master -> drive: 6040, 607A, 6060
    {0x1A01, 2, g_pdo_entries + 3}, // TxPDO: drive -> master: 6041, 6064
};

// Assign PDOs to Sync Managers.
ec_sync_info_t g_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, g_pdos + 0, EC_WD_DISABLE}, // SM2: RxPDO
    {3, EC_DIR_INPUT,  1, g_pdos + 1, EC_WD_DISABLE}, // SM3: TxPDO
    {0xff}
};

// Capacity for 12 axes * 5 PDO entries + one zero sentinel.
ec_pdo_entry_reg_t g_domain_regs[config::kMaxAxisCount * 5 + 1]{};

namespace {

void addReg(std::size_t& idx,
            uint16_t alias,
            uint16_t position,
            uint16_t object_index,
            uint8_t subindex,
            unsigned int* offset) {
    g_domain_regs[idx++] = ec_pdo_entry_reg_t{
        alias,
        position,
        config::kVendorId,
        config::kProductCode,
        object_index,
        subindex,
        offset,
        nullptr
    };
}

}  // namespace

void buildDomainRegsForActiveAxes() {
    // Clear the whole reserved table first. The final all-zero entry is the sentinel
    // required by ecrt_domain_reg_pdo_entry_list().
    for (auto& reg : g_domain_regs) {
        reg = ec_pdo_entry_reg_t{};
    }

    std::size_t idx = 0;
    for (int axis = 0; axis < config::kActiveAxisCount; ++axis) {
        const uint16_t alias = config::kAxisAliases[axis];
        const uint16_t position = config::kAxisPositions[axis];
        X5sPdoOffset& off = g_x5s_offsets[axis];

        addReg(idx, alias, position, 0x6040, 0x00, &off.control_word);
        addReg(idx, alias, position, 0x607A, 0x00, &off.target_position);
        addReg(idx, alias, position, 0x6060, 0x00, &off.mode_of_operation);
        addReg(idx, alias, position, 0x6041, 0x00, &off.status_word);
        addReg(idx, alias, position, 0x6064, 0x00, &off.position_actual);
    }
}
