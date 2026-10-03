## Electronics work plan

The electronics work follows the four project phases, split into A-1, A, B, C and D. Each phase's block diagram shows the subsystems built in that phase; dashed boxes mark the module or PDB that holds them.

**[Insert Phase A-1 block diagram here]**

Phase A-1 uses COTS parts for early locomotion and mechanism testing: a 2S battery, a 4-channel motor driver with built-in MCU, and an ESP32-C3 for Wi-Fi.

**[Insert Phase A block diagram here]**

Phase A needs basic locomotion under external command, and starts the custom PCB work. A custom PCB is needed because the SMORES package must fit many components, with onboard perception and compute as the end goal. Two boards:

- **Power board** — step-down regulator, LDO, four H-bridge drivers with current sensing.
- **Main board** — MCU for the H-bridges, IMU, UWB for inter-module ranging, Wi-Fi.

Both boards expose external GPIO for the components added in Phases B, C and D.

**[Insert Phase B block diagram here]**

Phase B needs module-to-module connection. The work is an in-wheel PCB with the EP-Face circuitry and the IR contact-alignment circuit. It is powered from the power board and talks to the main board through a slip ring, covered in the electrical connection section.

**[Insert Phase C block diagram here]**

Phase C adds onboard perception: an RGB-D/ToF camera, with the compute unit still wired externally. It also adds a wheel encoder, because the motor's built-in encoder sits before several gear stages and cannot sense wheel angle precisely.

**[Insert Phase D block diagram here]**

Phase D moves compute inside the robot: a Raspberry Pi Compute Module 5 on a custom carrier PCB, connected to the main board and the camera.

## Electronics component selection

Table 1 lists the selected components, their subsystem and the specifications behind the choice. Values are from datasheets; the EP-Face and IR board is custom and its figures come from the Phase B build.

**Table 1.** Selected electronics components by subsystem.

| Component | Subsystem and phase | Key specifications | Interface and supply |
|---|---|---|---|
| 4 × N20 gearmotor with encoder | Joint drive for the four DoF (left, right, pan, tilt); Phase A-1 onward | 6 V rated, 50 rpm output, 3 mm D-shaft, all-metal gearbox; magnetic quadrature encoder on the motor shaft, 7 PPR, upstream of the gearbox | A/B quadrature to the MCU; driven by H-bridge from the 6 V rail |
| STM32G474 | Main-board MCU: joint control, sensor sampling, data link; Phase A onward | Arm Cortex-M4F at 170 MHz, 512 KB flash, 128 KB SRAM; up to five 12-bit ADCs at 4 MSPS with timer-triggered conversion; high-resolution timer; three FDCAN controllers; CORDIC accelerator | SPI, I2C, UART, CAN; 1.71–3.6 V |
| ICM-42688-P | 6-axis IMU for attitude and orientation; Phase A onward | 3-axis gyroscope (±2000 dps) and 3-axis accelerometer (±16 g); ODR to 32 kHz; gyro noise 2.8 mdps/√Hz, accel noise 70 µg/√Hz; 2.5 × 3 × 0.9 mm | SPI (24 MHz) or I2C; 1.71–3.6 V |
| Qorvo (Decawave) DW3000 | UWB transceiver for inter-module ranging and coarse relative localisation; Phase A onward | IEEE 802.15.4z UWB; channels 5 (6.5 GHz) and 9 (8 GHz); two-way ranging accuracy of order 10 cm; data rate to 6.8 Mbps | SPI; 2.4–3.6 V |
| ESP32-C3 | Wi-Fi link; standalone controller in Phase A-1, wireless co-processor from Phase A | Single-core RISC-V at 160 MHz; Wi-Fi 4 (2.4 GHz) and Bluetooth 5 LE; 400 KB SRAM; 22 GPIO | UART or SPI to the main MCU; 3.0–3.6 V |
| EP-Face and IR board | In-wheel docking face: electropermanent magnet connector and IR contact-alignment sensing; Phase B | Custom PCB; EP magnet switched by a current pulse and holds with no standing power; IR emitter/receiver pairs for short-range alignment. Pulse current, holding force and IR range to be set by the Phase B design | Powered from the power board; data to the main board via slip ring |
| Raspberry Pi Compute Module 5 | Onboard compute for perception and planning; Phase D | Broadcom BCM2712, quad-core Arm Cortex-A76 at 2.4 GHz; 2–16 GB LPDDR4X; PCIe 2.0 ×1; two MIPI CSI/DSI ports; 40 × 55 mm | Custom carrier board; 5 V |
| Sipeed MaixSense A075V | RGB-D/ToF camera for onboard perception; Phase C onward | ToF depth 320 × 240 at 30 fps, range 0.2–2 m, 55°/72° field of view; RGB 800 × 600; onboard quad-core Cortex-A7 with 0.4 TOPS NPU; ROS 1/2 support | USB 2.0 or UART; 5 V |

## Electronics packaging: space-inside-the-robot feasibility analysis

All electronics must fit inside the robot at the final stage. We made rough CAD models of each component (battery, power board, main board, compute unit, camera, wheel encoder, in-wheel board) and placed them in the robot structure. Everything fits within the current mechanical design, so space is not expected to be a problem later in the project.

**[Insert Elec_Assembly image here]**  
**[Insert Elec_Exploded image here]**

## Additional scope not covered in the proposal: wheel encoder

The original SMORES sensed joint angle with a continuous-rotation potentiometer geared to the joint shaft. The wiper voltage gives the shaft angle directly, so position is absolute at power-up with no homing. The cost is coarse resolution (about 5°), contact wear, and backlash from the gear stage. The project is replacing this with a non-contact encoder.

Three options were considered; Table 2 compares them.

**Table 2.** Wheel-rotation sensing options compared.

| Criterion | On-axis magnetic rotary encoder (e.g. AS5600, AS5048) | Off-axis magnetic tape encoder (1 mm pole pitch) | Custom inductive encoder (PCB coil and inductance-to-digital IC) |
|---|---|---|---|
| Sensing principle | Diametrically magnetised magnet on the shaft end, read by a Hall-effect IC; angle from atan2(sin, cos) | Magnetised tape ring bonded to the wheel rim, read off-axis by a magnetoresistive or Hall read head at a fixed radial gap | PCB transmit and receive coils excite eddy currents in a conductive rotor target; phase and amplitude of the return signal encode angle |
| Output | Absolute over one turn | Incremental; an index or reference mark is needed to recover absolute position | Absolute over one turn, depending on coil layout |
| Resolution | 12–14 bit; below 0.1° with interpolation | Set by pole pitch and interpolation; medium to high | Set by coil geometry; comparable to magnetic encoders with interpolation |
| Mounting | Requires a free shaft end centred on the sensor IC | Anywhere on the rim; the shaft end stays free | Flat PCB facing the rotor target with a controlled sub-millimetre air gap |
| Immunity to magnetic fields | Sensitive to stray fields from the motors and the EP connector; needs shielding or standoff distance | Same exposure as on-axis, plus sensitivity to tape bonding and alignment error | Immune; unaffected by the motors or the EP connector |
| Design effort and cost | Low; off-the-shelf IC with mature firmware | Medium; tape must be applied and aligned precisely, and a dedicated read-head IC is needed | High; custom coil design, rotor target machining and calibration, with no drop-in part |
| Fit for this project | Simplest and lowest risk, but the SMORES layout already uses the wheel shaft end for gearing and mounting, so a clear axial spot must be reserved | Frees the shaft end, but adds a homing step at power-up and a wear and bonding risk on the tape | Best fit to the EP connector's nearby field, but the highest design time and schedule risk within the capstone timeline |

The EP connector's field is close to the wheel sensor, so the two magnetic options carry an interference risk the inductive option does not. Weighing that against the extra design effort is the open decision for Phase C.

## Current focus: robot locomotion

### Power system

The budget is built in three steps: the current each component draws (Table 3), the average load over a mission once duty cycles are applied (Table 4), and the power drawn from the pack after regulator losses, which gives the run time (Table 5).

**Component loads.** Table 3 gives the rail, typical current and maximum current for each component. Typical is normal duty; maximum is the worst case the Power IC must survive.

**Table 3.** Component current by rail.

| Component | Rail | Typical current | Maximum current | Condition |
|---|---|---|---|---|
| 4 × N20 gearmotor (6 V, 50 rpm) | Pack, PWM-limited to 6 V | 0.07 A each; 0.28 A for four | 1.0 A each; 4.0 A for four | Typical is no-load; maximum is all four stalled, at 6 V equivalent |
| STM32G474 | 3.3 V | 30 mA | 60 mA | 170 MHz; maximum with all peripherals active |
| ICM-42688-P | 1.8 V (3.3 V I/O) | 0.9 mA | 1.2 mA | Six-axis low-noise mode |
| DW3000 * | 3.3 V | 45 mA | 50 mA | Listening; transmit peak. Idle about 10 mA, deep sleep below 1 µA |
| ESP32-C3 | 3.3 V | 90 mA | 335 mA | Wi-Fi active; maximum is 802.11b transmit. Modem sleep 20 mA |
| EP-Face and IR board * | - | - | Need to be calculated from EP design | Steady draw is IR only; the EP magnet holds with no current. The pulse comes from a local capacitor, so the rail only sees recharge current |
| Raspberry Pi Compute Module 5 * | 5 V | 1.2 A | 3.0 A | Idle about 0.6 A; maximum is full CPU load, no USB peripherals |
| Sipeed MaixSense A075V * | 5 V | 0.4 A | 0.5 A | ToF illuminator active at 30 fps |

**Mission average.** Table 4 applies a duty cycle to each component over a representative mission: mostly driving, intermittent ranging, occasional docking, perception always on.

**Table 4.** Estimated average load over a representative mission.

| Component | Rail | Active current | Duty cycle | Average current at rail | Average power |
|---|---|---|---|---|---|
| 2 × N20 wheel motors | Pack, PWM to 6 V | 0.25 A each (loaded, est.) | 60 % (driving) | 0.30 A | 1.80 W |
| 2 × N20 pan and tilt motors | Pack, PWM to 6 V | 0.25 A each (loaded, est.) | 15 % (reconfiguring) | 0.075 A | 0.45 W |
| STM32G474 | 3.3 V | 30 mA | 100 % | 30 mA | 0.10 W |
| ICM-42688-P | 1.8 V | 0.9 mA | 100 % | 0.9 mA | < 0.01 W |
| DW3000 | 3.3 V | 45 mA listening; 10 mA idle | 25 % ranging, 75 % idle | 19 mA | 0.06 W |
| ESP32-C3 | 3.3 V | 90 mA active; 20 mA modem sleep | 60 % active, 40 % sleep | 62 mA | 0.20 W |
| EP-Face and IR board | 3.3 V | 35 mA | 20 % docking approach | 7 mA | 0.02 W |
| Raspberry Pi Compute Module 5 | 5 V | 1.2 A | 100 % | 1.2 A | 6.00 W |
| Sipeed MaixSense A075V | 5 V | 0.4 A | 80 % | 0.32 A | 1.60 W |
| **Total at the rails** | | | | | **10.2 W** |



**Table 5.** Power drawn from the pack by rail, including converter efficiency.

| Rail | Load at rail | Conversion from pack | Power from pack |
|---|---|---|---|
| Motors (pack, PWM to 6 V) | 2.25 W | H-bridge, 95 % | 2.4 W |
| 5 V (compute, camera) | 7.60 W | Buck, 90 % | 8.4 W |
| 3.3 V and 1.8 V (MCU, sensors, radios) | 0.38 W | LDO from 5 V, 66 %, then buck, 90 % | 0.65 W |
| **Total from pack** | | | **11.5 W** |

**Battery run time.** The pack is two 505080 cells in series, 3000 mAh, 22.2 Wh, of which 17.8 Wh is usable to cutoff. 11.5 W from the pack is 1.55 A at 7.4 V, about 0.5 C. Run time is about **1.5 h** for the full Phase D module; the Phase A and B configuration, without compute and camera (73 % of the draw), runs about **5.8 h**.

**Power board components**

**[Insert power block diagram here]**

Power splits into three paths from the 2S pack:

- **Motors.** The four N20s run from the pack through the H-bridges. PWM duty is capped at about 70 % (6 V over 8.4 V at full charge) to hold the motors at their 6 V rating, and the H-bridge current sensing trips on stall. The on-time current at stall scales to about 1.4 A, so the H-bridges are rated for the pack voltage and 1.5 A per channel.
- **EP-Face and IR.** A dedicated step-down regulator feeds the in-wheel board and its pulse capacitor. The pulse itself comes from the capacitor; the separate regulator keeps the recharge current and pulse transients off the logic rail so the MCU and compute unit do not brown out during docking.
- **Logic and compute.** A second step-down regulator provides 5 V to the main board, encoder circuit, compute unit and camera; 3.3 V for the MCU, sensors and radios comes from an LDO on the main board. This rail carries about 1.6 A typical and 4 A peak (Table 3), so the regulator is rated 5 V / 5 A, which also meets the Compute Module 5 supply requirement.


**Table 6.** Maximum load per rail and the resulting regulator requirement.

| Rail | Loads | Maximum current | Maximum power |
|---|---|---|---|
| Motor drive (pack, PWM to 6 V) | 4 × N20 | 4.0 A at 6 V equivalent; 1.4 A per channel on-time at 8.4 V | 24 W |
| 5 V logic and compute | CM5 3.0 A, camera 0.5 A, 3.3 V LDO input 0.45 A | 4.0 A | 20 W |
| 3.3 V (from 5 V) | STM32 60 mA, IMU 1 mA, DW3000 50 mA, ESP32-C3 335 mA | 0.45 A (ESP32 transmit burst) | 1.5 W |
| EP-Face and IR |  |  | TBD |
| **Pack total** | all rails at peak | ≈ 6.5 A at 7.4 V, ≈ 7.3 A at 6.6 V | ≈ 47 W |