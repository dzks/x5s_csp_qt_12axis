#pragma once
#include <cstdint>

enum class CiA402State { SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn, OperationEnabled, Fault, Unknown };

CiA402State parseStatusWord(uint16_t status_word);
uint16_t baseControlWordForState(uint16_t status_word);
bool isOperationEnabled(uint16_t status_word);
const char* cia402StateName(CiA402State state);
