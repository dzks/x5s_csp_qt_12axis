#pragma once
#include <array>
#include <cstdint>

namespace config {

constexpr unsigned int kMasterIndex = 0;

// -----------------------------------------------------------------------------
// Multi-axis capacity configuration
// -----------------------------------------------------------------------------
// kMaxAxisCount is the reserved code capacity. The program allocates offset,
// command, status, and Qt table space for 12 axes.
constexpr int kMaxAxisCount = 12;

// kActiveAxisCount is the number of physically connected/configured slaves that
// the program will actually request, configure, register into the PDO domain,
// and control. This version is a 2-axis program.
//
// When adding more X5S drives later, first connect them in EtherCAT order, then
// change only this value if all drives are the same model and use positions 0..N-1.
constexpr int kActiveAxisCount = 6;
static_assert(kActiveAxisCount >= 1, "At least one axis must be active.");
static_assert(kActiveAxisCount <= kMaxAxisCount, "kActiveAxisCount exceeds kMaxAxisCount.");

// Alias is normally 0 unless you explicitly configured EtherCAT aliases.
constexpr std::array<uint16_t, kMaxAxisCount> kAxisAliases = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// Physical EtherCAT slave positions. IgH positions start from 0 in bus order.
// The first two positions are used now; positions 2..11 are reserved.
constexpr std::array<uint16_t, kMaxAxisCount> kAxisPositions = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};

// Current assumption: all 12 reserved slots are the same X5S model.
constexpr uint32_t kVendorId = 0x00000766;
constexpr uint32_t kProductCode = 0x00010000;

// 1 ms EtherCAT/CSP control cycle.
constexpr int64_t kCycleTimeNs = 1000000;

// This version is CSP-only: 6060 is written as CSP from the beginning.
constexpr int16_t kModeCSP = 8;

// Qt UI default motion parameters.
constexpr int32_t kDefaultStepCountsPerCycle = 1;
constexpr int kWaitCyclesAfterOperationEnabled = 100;  // about 100 ms before accepting a new move

// CSP on this X5S setup should keep DC enabled.
constexpr bool kEnableDcConfig = true;
constexpr uint32_t kDcAssignActivate = 0x0300;

// Status printing / GUI refresh helper.
constexpr int kStatusPrintPeriodCycles = 500;

}  // namespace config
