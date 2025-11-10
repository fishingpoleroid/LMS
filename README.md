# LMS
# ESP32 Line Maze Solver

> A complete, modular, competition-ready line-maze solver using an ESP32, QTR-8RC sensor array, L298N driver, and N-20 encoders.

This repository contains a production-quality implementation and explanation for a two-run maze solving strategy used in robotics competitions (mapping + optimized solving). The README below explains the architecture, wiring, code structure, configuration steps, and tuning advice — all derived from the project design notes and implementation in the provided documentation.

---

## Table of Contents

* [Overview](#overview)
* [Key Features](#key-features)
* [Hardware](#hardware)
* [Wiring & Power](#wiring--power)
* [ESP32 Pin Assignments (Pins.h)](#esp32-pin-assignments-pinsh)
* [Software Architecture & File Structure](#software-architecture--file-structure)
* [How It Works (High-level Flow)](#how-it-works-high-level-flow)

  * [Run 1: Mapping (LSRB)](#run-1-mapping-lsrb)
  * [Path Optimization (Run 1.5)](#path-optimization-run-15)
  * [Run 2: Solving (Optimized Run)](#run-2-solving-optimized-run)
* [Modules & Responsibilities](#modules--responsibilities)

  * [Sensors (Sensors.h / Sensors.cpp)](#sensors-sensorsh--sensorscpp)
  * [Motors & Encoders (Motors.h / Motors.cpp)](#motors--encoders-motorsh--motorscpp)
  * [Path Optimization](#path-optimization)
  * [Main FSM (main.cpp / .ino)](#main-fsm-maincpp--ino)
* [PID Line Following & Tuning](#pid-line-following--tuning)
* [Calibration Procedure](#calibration-procedure)
* [Compiling & Uploading](#compiling--uploading)
* [Tuning Checklist & Troubleshooting](#tuning-checklist--troubleshooting)
* [References & Credits](#references--credits)
* [License](#license)

---

## Overview

This project implements a robust line maze solver optimized for competitive robotics. It separates responsibilities into modular subsystems (Sensors, Motors, PathOptimization, and the main controller). The robot performs a long mapping run using LSRB (Left-Straight-Right-Back), records the raw path, optimizes that path using deterministic replacement rules, and then executes the optimized path at higher speed for the final solved run.

> The design prioritizes reliability: analog (PID) control for continuous line tracking, and discrete encoder-driven turns for repeatable intersections.

---

## Key Features

* QTR-8RC reflectance array integration and robust 10s calibration routine
* ESP32-ledc PWM motor control for smooth, high-frequency PWM (5 kHz, 8-bit)
* Encoder-backed, hardware PCNT/ESP32Encoder usage for zero-CPU-overhead counting
* Atomic encoder-based pivot turns (90°, 180°) and precise forward moves to center intersections
* LSRB mapping and iterative string-replacement path optimization
* Finite State Machine (FSM) that cleanly separates calibration, mapping, optimizing, and solving

---

## Hardware

Minimum hardware to replicate this project:

* ESP32 (any dev board compatible with ESP32 Arduino core)
* Pololu QTR-8RC reflectance sensor array
* L298N dual H-bridge motor driver
* Two N-20 DC motors with incremental encoders
* 12V battery pack (for motors) and LM2596 buck converter to produce stable 5V for logic
* Misc: push-button (start), jumper wires, chassis, wheels

**Important note:** Do not use the L298N on-board 5V regulator. Instead, use a separate high-quality buck converter (e.g., LM2596) for logic 5V to avoid noise and brownouts.

---

## Wiring & Power

1. **12V primary rail:** Connect battery positive to L298N VMS (motor supply). Also feed LM2596 input.
2. **5V logic rail:** Use LM2596 to step 12V down to a stable 5V. Use this for:

   * ESP32 Vin (or 5V pin depending on your board)
   * QTR-8RC VCC
   * L298N VCC (logic)
3. **Ground:** Share ground between the battery, LM2596, ESP32, L298N, and sensors.

This separation isolates motor-current spikes from the sensitive logic rail.

---

## ESP32 Pin Assignments (Pins.h)

A single header (`Pins.h`) centralizes all hardware pin mappings. Example (right-most sensor = QTR_PIN_1):

```c++
// QTR-8RC
#define QTR_PIN_1 36
#define QTR_PIN_2 39
#define QTR_PIN_3 34
#define QTR_PIN_4 35
#define QTR_PIN_5 32
#define QTR_PIN_6 33
#define QTR_PIN_7 25
#define QTR_PIN_8 26
#define QTR_EMITTER_PIN 27

const uint8_t SensorCount = 8;

// Motors (L298N)
#define MOTOR_L_IN1 23
#define MOTOR_L_IN2 22
#define MOTOR_L_ENA 21 // PWM
#define MOTOR_R_IN3 19
#define MOTOR_R_IN4 18
#define MOTOR_R_ENB 5  // PWM

// Encoders
#define ENCODER_L_A 17
#define ENCODER_L_B 16
#define ENCODER_R_A 4
#define ENCODER_R_B 15

#define ONBOARD_LED 2
#define USER_BUTTON 0
```

> The chosen pins avoid conflicting with ESP32 boot strapping behavior and use ADC/PCNT-capable pins for sensors and encoders where appropriate.

---

## Software Architecture & File Structure

A recommended modular layout:

```
/src
  |- Pins.h
  |- Sensors.h
  |- Sensors.cpp
  |- Motors.h
  |- Motors.cpp
  |- PathOptimization.h
  |- PathOptimization.cpp
  |- main.cpp (.ino)
/lib
  |- QuickPID (or install via Library Manager)
  |- ESP32Encoder
  |- QTRSensors
README.md
```

Each module exposes a small, well-documented API so the main FSM can be compact and clear.

---

## How It Works (High-level Flow)

The robot uses a Finite State Machine with states: `CALIBRATING`, `WAIT_FOR_RUN_1`, `MAPPING`, `OPTIMIZING`, `WAIT_FOR_RUN_2`, `SOLVING`, `FINISHED`.

### Run 1: Mapping (LSRB)

1. Follow line using a PID loop (analog control).
2. On intersection detection, perform `stopBrake()` then `moveForward(TICKS_TO_CENTER)` to place robot center over junction.
3. Attempt paths in the LSRB order:

   * Turn left; if line present, record 'L'
   * Else return center, try Straight; if present, record 'S'
   * Else try Right; if present, record 'R'
   * Else it's a dead end: perform 180 and record 'B'
4. Continue until `isLineEnd()` returns true; raw path string contains the recorded sequence.

### Path Optimization (Run 1.5)

To remove inefficient maneuvers, apply iterative replacement rules until no change occurs. Example rules:

* `LBR` -> `B`
* `LBS` -> `R`
* `RBL` -> `B`
* `SBL` -> `R`
* `SBS` -> `B`
* `LBL` -> `S`

These are applied repeatedly since one replacement can enable another.

### Run 2: Solving (Optimized Run)

Load the optimized path and execute commands at each intersection: `L`, `S`, `R`. 'B' should not exist after optimization. Use the same PID line-following for straight segments and encoder-based turns for intersections.

---

## Modules & Responsibilities

### Sensors (Sensors.h / Sensors.cpp)

* Wrap Pololu `QTRSensors` API for initialization, calibration, and sensory helpers.
* `setup()` performs a robust 10s calibration sweep (repeated calls to `qtr.calibrate()`), prints calibration min/max.
* `int16_t getLineError()` returns a normalized error in range [-3500, +3500] using `readLineBlack()`.
* `bool isIntersection()` checks raw sensor values (e.g. both outermost sensors seeing black) and detects T/X/90° junctions.
* `bool isLineEnd()` detects dead ends or finish line (all sensors read white below a threshold).

**Calibration thresholds** cited in implementation: black > 800, white < 200 (tweak per environment).

### Motors & Encoders (Motors.h / Motors.cpp)

* Use ESP32 `ledc` peripheral for PWM (5 kHz, 8-bit) instead of `analogWrite()` for better performance.
* Drive L298N direction pins and attach PWM to ENA/ENB.
* Use `ESP32Encoder` library, which leverages the hardware PCNT for low-overhead counting. Attach encoders with `attachHalfQuad(pinA, pinB)` and poll counts with `getCount()`.
* Provide `setSpeeds(left, right)` with range [-255, +255], and `stopBrake()` that sets both input pins HIGH for fast braking.
* Provide atomic blocking encoder-based functions: `turn_90_left()`, `turn_90_right()`, `turn_180_back()`, and `moveForward(ticks)`.

*Tuning constants to set in code:*

```c++
#define TICKS_FOR_90_DEG 450        // tune for your robot
#define TICKS_FOR_180_DEG (TICKS_FOR_90_DEG * 2)
#define TICKS_TO_CENTER 150         // move forward to center on junction
```

### Path Optimization

A small string-manipulation module runs replacement rules in a loop until no further changes are possible.

Implementation details:

* Safely copy path string into a temporary buffer each pass.
* Apply each replacement (e.g., `replaceSubstring(temp, "LBR", "B")`).
* Repeat until `strcmp(path, temp) == 0`.

### Main FSM (main.cpp / .ino)

* Initialize Serial, subsystems, and PID controller in `setup()`.
* `sensors.setup()` performs 10s calibration and prints results.
* Use a button for `WAIT_FOR_RUN_1` and `WAIT_FOR_RUN_2` to let user position robot at START.
* Main `loop()` contains the state machine and the core `runPID()` function used by both runs.

---

## PID Line Following & Tuning

The PID controller expects `pidInput` as the signed line error (-3500..+3500) and computes `pidOutput` used to adjust motor speeds:

```
rightSpeed = baseSpeed + correction
leftSpeed  = baseSpeed - correction
```

Suggested starting PID constants (must be tuned on your setup):

```c++
float Kp = 0.05, Ki = 0.0001, Kd = 0.02;
int baseSpeed = 150; // 0..255
```

Tuning tips:

* Increase `Kp` for responsiveness; reduce if oscillation occurs.
* Increase `Ki` slowly to correct bias (unequal motors) — watch for windup.
* `Kd` damps rapid moves; increase if overshoot is frequent.
* Start with `baseSpeed` low while tuning (e.g. 80–120), then increase for Run 2 once stable.

---

## Calibration Procedure

1. Power the robot and open Serial Monitor at 115200 baud.
2. During setup, the robot will run a 10s calibration routine: sweep the QTR sensors across line + background.
3. Confirm `Calibrated Minimums:` and `Calibrated Maximums:` printed in Serial.
4. If readings look incorrect, re-run calibration in a more controlled sweep and ensure emitter is turned on.

---

## Compiling & Uploading

* Install these Arduino libraries via Library Manager (or PlatformIO equivalents): `QTRSensors`, `ESP32Encoder`, `QuickPID`.
* Select the correct ESP32 board and COM port.
* Build and upload using the Arduino IDE or PlatformIO.

**PlatformIO tip:** Create a `platformio.ini` with the proper board identifier and include the library dependencies.

---

## Tuning Checklist & Troubleshooting

* Check wiring and 5V supply (no brownouts). Use a high-quality buck converter for logic 5V.
* Verify QTR emitter pin is enabled and sensors are physically aligned at consistent height above the track.
* Ensure encoders give consistent counts when rotating wheels by hand.
* If encoder counts are noisy, confirm pull-ups and use ESP32Encoder recommended pin pairs.
* If robot drifts left or right, adjust PID `Ki` or rebalance base motor speed.
* If intersections are missed, lower the intersection detection threshold or re-check sensor calibration.

---
