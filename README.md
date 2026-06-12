# X5S IgH EtherCAT CSP Qt Multi-Axis Demo

This version is modified from the original single-axis project.

Current target:

- Active axes: 2
- Reserved code/UI capacity: 12 axes
- Control mode: CSP only, `6060 = 8`
- Cycle time: 1 ms
- DC: enabled by default
- UI: Qt Widgets

## Build

```bash
mkdir -p build
cd build
cmake ..
make -j
```

## Run Qt UI

```bash
sudo ./x5s_csp_qt
```

## Run console fallback

```bash
sudo ./x5s_csp_console_direct
```

Console commands:

```text
m <axis> <target> <step>   e.g. m 0 1000 1
h <axis>                   hold one axis
ha                         hold all active axes
q                          quit
```

## Important EtherCAT checks before running

```bash
sudo ethercat slaves
sudo ethercat slaves -p 0 -v
sudo ethercat slaves -p 1 -v
sudo ethercat pdos -p 0
sudo ethercat pdos -p 1
sudo ethercat upload -p 0 -t uint32 0x1018 1
sudo ethercat upload -p 0 -t uint32 0x1018 2
sudo ethercat upload -p 1 -t uint32 0x1018 1
sudo ethercat upload -p 1 -t uint32 0x1018 2
```

`1018:01` is Vendor ID. `1018:02` is Product Code.

## Files changed for multi-axis support

### `include/config.hpp`

This is the main file you edit when adding axes.

Current settings:

```cpp
constexpr int kMaxAxisCount = 12;
constexpr int kActiveAxisCount = 2;
constexpr std::array<uint16_t, kMaxAxisCount> kAxisAliases = {0, ...};
constexpr std::array<uint16_t, kMaxAxisCount> kAxisPositions = {0, 1, 2, ..., 11};
constexpr uint32_t kVendorId = 0x00000766;
constexpr uint32_t kProductCode = 0x00010000;
```

If all drives are the same X5S model and connected in normal bus order, adding a third axis usually only requires:

```cpp
constexpr int kActiveAxisCount = 3;
```

The reserved position table already contains position 2.

### `include/pdo_config.hpp` / `src/pdo_config.cpp`

Single-axis:

```cpp
X5sPdoOffset g_x5s_offset;
```

Multi-axis:

```cpp
std::array<X5sPdoOffset, config::kMaxAxisCount> g_x5s_offsets;
```

`g_domain_regs` has reserved capacity for 12 axes, but `buildDomainRegsForActiveAxes()` only registers `kActiveAxisCount` axes. This is important: do not register disconnected reserved axes, or EtherCAT initialization may fail.

### `include/ethercat_master.hpp` / `src/ethercat_master.cpp`

Single-axis:

```cpp
ec_slave_config_t* slave_config_;
```

Multi-axis:

```cpp
std::array<ec_slave_config_t*, config::kMaxAxisCount> slave_configs_;
```

The master now loops through every active axis, calls:

```cpp
ecrt_master_slave_config(...)
ecrt_slave_config_pdos(...)
ecrt_slave_config_dc(...)
```

for each active slave.

### `include/x5s_axis.hpp` / `src/x5s_axis.cpp`

`X5sAxis` no longer uses the global single-axis `g_x5s_offset` directly. It receives its own offset pointer:

```cpp
X5sAxis(master.domainData(), &g_x5s_offsets[axis], axis);
```

This is the key change that lets the same class control axis 0, axis 1, ... axis 11.

### `include/shared_state.hpp`

Qt commands and status are now arrays sized by `kMaxAxisCount`.

The first `kActiveAxisCount` entries are active. The remaining entries are shown as reserved.

### `src/control_worker.cpp`

The 1 ms control loop now uses:

```cpp
std::vector<X5sAxis> axes;
```

and loops over all active axes for:

- read PDO
- consume Qt command
- write CSP mode
- hold before OP/enabled
- start/update CSP target
- write PDO
- update UI status

### `src/main_qt.cpp`

Qt UI now has:

- one axis selector for active axes
- target/step input for selected axis
- hold selected axis
- hold all active axes
- a 12-row status table showing active and reserved slots

## Adding more axes later

For the normal case where every drive is the same X5S model, uses alias 0, and is connected in bus order:

1. Physically connect the new drive.
2. Confirm it appears:

```bash
sudo ethercat slaves
```

3. Confirm its position and identity:

```bash
sudo ethercat slaves -p 2 -v
sudo ethercat upload -p 2 -t uint32 0x1018 1
sudo ethercat upload -p 2 -t uint32 0x1018 2
```

4. Edit only:

```cpp
// include/config.hpp
constexpr int kActiveAxisCount = 3;
```

5. Rebuild.

If the physical position is not the same as the logical axis index, edit:

```cpp
kAxisPositions
```

For example, if logical axis 2 should control EtherCAT slave position 5:

```cpp
constexpr std::array<uint16_t, kMaxAxisCount> kAxisPositions = {
    0, 1, 5, 3, 4, 5, 6, 7, 8, 9, 10, 11
};
```

If future drives have a different Vendor ID or Product Code, the current code assumes one common model. You should then extend `config.hpp`, `pdo_config.cpp`, and `ethercat_master.cpp` to use per-axis vendor/product arrays.

## Safety note

This program writes CSP targets every 1 ms. Before commanding motion:

- keep the motor unloaded or safely mounted
- verify positive/negative direction
- start with small target values and small step values
- make sure emergency stop and drive enable safety are available
