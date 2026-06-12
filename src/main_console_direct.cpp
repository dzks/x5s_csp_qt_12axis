#include "config.hpp"
#include "control_worker.hpp"
#include "shared_state.hpp"

#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

static SharedState* g_shared = nullptr;

void signalHandler(int) {
    if (g_shared) g_shared->stop();
}

int main() {
    SharedState shared_state;
    g_shared = &shared_state;
    std::signal(SIGINT, signalHandler);

    std::thread worker([&shared_state]() { runControlLoop(shared_state); });

    std::cout << "[INFO] Console direct-CSP multi-axis test. active="
              << config::kActiveAxisCount << " reserved=" << config::kMaxAxisCount << "\n";
    std::cout << "[INFO] Commands:\n";
    std::cout << "  m <axis> <target> <step>   e.g. m 0 1000 1\n";
    std::cout << "  h <axis>                   hold one axis\n";
    std::cout << "  ha                         hold all active axes\n";
    std::cout << "  q                          quit\n";

    while (shared_state.isRunning()) {
        std::string cmd;
        if (!(std::cin >> cmd)) break;

        if (cmd == "q" || cmd == "Q") {
            shared_state.stop();
            break;
        }

        if (cmd == "ha" || cmd == "HA") {
            shared_state.requestHoldAllActiveAxes();
            continue;
        }

        if (cmd == "h" || cmd == "H") {
            int axis = 0;
            std::cin >> axis;
            shared_state.requestHold(axis);
            continue;
        }

        if (cmd == "m" || cmd == "M") {
            int axis = 0;
            int32_t target = 0;
            int32_t step = config::kDefaultStepCountsPerCycle;
            std::cin >> axis >> target >> step;
            shared_state.requestTarget(axis, target, step);
            continue;
        }

        std::cout << "Invalid command. Use: m <axis> <target> <step>, h <axis>, ha, or q.\n";
    }

    shared_state.stop();
    if (worker.joinable()) worker.join();
    return 0;
}
