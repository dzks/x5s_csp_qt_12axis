#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
mkdir -p build
cd build
cmake ..
make -j
sudo ./x5s_csp_console_direct
