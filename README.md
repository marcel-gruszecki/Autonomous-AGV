# Autonomous AGV Line Follower — ESP32

A two-wheeled autonomous guided vehicle (AGV) built around a custom PCB design,
hardware assembly, and embedded firmware. The project covers the full development
cycle, from the electrical schematic to working code on the target hardware.

The mechanical platform comes from the *OSOYOO 2WD Robot Car Starter Kit for Arduino*,
which was used as a chassis and drivetrain base only. The stock Arduino Uno and its
motor shield were removed and replaced with a self-designed ESP32-WROOM carrier board,
adding Wi-Fi connectivity and a 32-bit MCU in place of the original 8-bit controller.

![Assembled robot with the custom PCB](Photo.jpg)

## Hardware

**Custom PCB (KiCad, manufactured by PCBWay)** — a single-board carrier designed from
scratch, integrating:

- Power section based on LM2596 step-down converters (7.4 V Li-ion pack → 5 V / 3.3 V rails)
- Separate power distribution path for the motor driver
- Connectors for a three-channel TCRT5000 optical line-sensor array
- ESP32-WROOM main circuit with an optimised keepout zone under the RF antenna

**Drivetrain and sensing** — 2× DC gear motors driven through a Model-X dual H-bridge
(PWM speed control on ENA/ENB), 3× TCRT5000 reflective sensors for line detection.

![Electrical schematic](Scheme.png)

*Electrical schematic: battery and power conversion, ESP32-WROOM module, motor
driver interface, and the line-sensor array (KiCad).*

The board was assembled by hand (T12 soldering station), including component selection,
mechanical integration, and debugging of short-circuit and polarity faults on the first
revision.

## Firmware and tooling

- `OSOYOO/` — ESP32 firmware, C++ on the Arduino framework (PlatformIO)
- `Serwer_rust/` — host-side control client in Rust (standard library only)
- `Scheme/` — KiCad schematic, PCB layout, and Gerber files

The ESP32 joins a Wi-Fi network, broadcasts its address over UDP for discovery, and
exposes a line-based TCP command interface with a motion watchdog that stops the
drive if the link goes quiet. The Rust client discovers the robot automatically and
provides a manual console for commanding the motors and reading the sensor array.

> **Project status: work in progress.** The current firmware and client are a
> deliberately minimal bring-up harness — their purpose is to validate wiring, PWM
> output, sensor readings, and the Wi-Fi link on the first PCB revision. The
> line-following control logic is not implemented yet.

## Roadmap

The single robot shown above is the first unit of a planned three-vehicle fleet.

1. **Line-following control** — state machine on the ESP32, deterministic processing of
   the optical sensor signals, and real-time drive control.
2. **Fleet of three robots** — two further units built from the same PCB revision, each
   individually addressable over Wi-Fi.
3. **Warehouse test bed** — a self-built scale warehouse mock-up (taped route network,
   junctions, pick-up and drop-off stations) used to test navigation, multi-vehicle
   traffic on shared routes, and collision avoidance. The layout is designed in CAD
   ahead of construction:

   ![CAD design of the warehouse test bed](Warehouse.png)

   *CAD model of the warehouse mock-up: route network, junctions, and station layout.*

4. **Coordination layer** — extending the Rust host application from a manual console
   into a supervisor that dispatches transport jobs to the fleet.

## Build and run

```bash
# Firmware
cd OSOYOO
pio run -t upload            # set WIFI_SSID / WIFI_PASS in src/main.cpp first

# Host client
cd Serwer_rust
cargo run                    # auto-discovers the robot on the local network
```

Command interface (`M <left> <right>`, `S`, `SENS`, `INFO`, `PING`) is documented in
the firmware header.

## Tech stack

KiCad · PCBWay · ESP32-WROOM · C++ / Arduino · PlatformIO · Rust · TCP/UDP
