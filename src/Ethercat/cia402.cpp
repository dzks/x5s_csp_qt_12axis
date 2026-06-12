#include "Ethercat/cia402.hpp"

CiA402State parseStatusWord(uint16_t sw) {
    if (sw & 0x0008) return CiA402State::Fault;
    const uint16_t masked = sw & 0x006F;
    if ((sw & 0x004F) == 0x0040) return CiA402State::SwitchOnDisabled;
    if (masked == 0x0021) return CiA402State::ReadyToSwitchOn;
    if (masked == 0x0023) return CiA402State::SwitchedOn;
    if (masked == 0x0027) return CiA402State::OperationEnabled;
    return CiA402State::Unknown;
}

uint16_t baseControlWordForState(uint16_t sw) {
    switch (parseStatusWord(sw)) {
        case CiA402State::Fault: return 0x0080;
        case CiA402State::SwitchOnDisabled:
        case CiA402State::Unknown: return 0x0006;
        case CiA402State::ReadyToSwitchOn: return 0x0007;
        case CiA402State::SwitchedOn: return 0x000F;
        case CiA402State::OperationEnabled: return 0x000F;
        default: return 0x0006;
    }
}

bool isOperationEnabled(uint16_t sw) { return parseStatusWord(sw) == CiA402State::OperationEnabled; }

const char* cia402StateName(CiA402State state) {
    switch (state) {
        case CiA402State::SwitchOnDisabled: return "SwitchOnDisabled";
        case CiA402State::ReadyToSwitchOn: return "ReadyToSwitchOn";
        case CiA402State::SwitchedOn: return "SwitchedOn";
        case CiA402State::OperationEnabled: return "OperationEnabled";
        case CiA402State::Fault: return "Fault";
        case CiA402State::Unknown: return "Unknown";
        default: return "Invalid";
    }
}
