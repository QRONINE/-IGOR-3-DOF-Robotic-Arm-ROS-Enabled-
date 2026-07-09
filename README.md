# IGOR

<p align="center">
  <img src="docs/images/IGOR-FRONT.png" width="47%" alt="IGOR Robotic Manipulator Front View">
  <img src="docs/images/IGOR-SIDE.png" width="47%" alt="IGOR Robotic Manipulator Side View">
</p>

<h3 align="center">
A Vision-Guided ROS enabled 3-DOF Robotic Manipulator with Distributed Embedded Motion Control
</h3>

<p align="center">
ESP32 • Arduino Uno • GRBL • CNC Shield V3 • TMC Stepper Drivers • NEMA 17 • AS5600 Encoders • UART • I²C • SolidWorks • ROS 2 • Computer Vision • Stockfish
</p>

---

## Project Overview

**IGOR** is a custom 3-DOF robotic manipulator designed and developed as a senior-year engineering capstone project.

The project began as an attempt to design a low-cost robotic arm capable of accurately positioning chess pieces on a physical chessboard.

The first prototype, **IGOR V1**, established the mechanical structure, motorized joints, custom transmission systems, encoder integration, and basic embedded motion control.

The second-generation system, **IGOR V2**, redesigned the control architecture around a distributed embedded system.

Instead of using a single microcontroller to perform all sensing, control, and motion-generation tasks, IGOR V2 separates the system into two primary control layers:

* an **ESP32 supervisory controller**
* an **Arduino Uno running GRBL as the real-time motion-control layer**

The ESP32 handles:

* supervisory control
* encoder acquisition
* I²C peripheral communication
* OLED system monitoring
* high-level motion commands
* UART communication with the motion controller
* future kinematic computation
* future micro-ROS communication

The Arduino Uno, CNC Shield V3, and GRBL firmware handle:

* deterministic step generation
* motor acceleration profiles
* coordinated multi-axis movement
* homing
* limit-switch handling
* soft limits
* motion buffering
* G-code execution

Three NEMA 17 stepper motors drive the manipulator through custom mechanical transmission systems.

The joints use TMC-series stepper drivers configured for **1/16 microstepping**, significantly improving motion smoothness and reducing mechanical vibration compared with the earlier driver configuration.

Joint position is measured using **AS5600 12-bit magnetic encoders** connected to the ESP32 through a **TCA9548A I²C multiplexer**.

The manipulator currently operates as a functional embedded robotic platform with:

* three motorized joints
* GRBL-based motion control
* distributed ESP32/Arduino architecture
* UART communication
* encoder monitoring
* OLED system feedback
* homing
* limit switches
* configurable motion profiles
* microstepping
* custom transmissions
* custom mechanical design
* improved motion smoothness
* reduced vibration
* higher motion reliability

The long-term objective is to integrate the manipulator with a computer-vision chess pipeline and the Stockfish chess engine to create an autonomous physical chess-playing robotic system.

The project is being developed through the following engineering process:

> **Requirements → Mechanical Design → Electronics Architecture → Motion Control → Firmware → Integration → Failure → Debugging → Optimization → Validation → Future Architecture**

---

# Table of Contents

1. [Current System](#1-current-system)
2. [Project Objectives](#2-project-objectives)
3. [System Specifications](#3-system-specifications)
4. [System Architecture](#4-system-architecture)
5. [IGOR V1](#5-igor-v1)
6. [Why IGOR V2 Was Developed](#6-why-igor-v2-was-developed)
7. [IGOR V2 Control Architecture](#7-igor-v2-control-architecture)
8. [Repository Structure](#8-repository-structure)
9. [Bill of Materials](#9-bill-of-materials)
10. [Tools Required](#10-tools-required)
11. [Mechanical Design](#11-mechanical-design)
12. [Joint Transmission Architecture](#12-joint-transmission-architecture)
13. [Stepper Motors](#13-stepper-motors)
14. [TMC Stepper Drivers](#14-tmc-stepper-drivers)
15. [Microstepping](#15-microstepping)
16. [Power-System Architecture](#16-power-system-architecture)
17. [Power Distribution](#17-power-distribution)
18. [Arduino Motion-Control Layer](#18-arduino-motion-control-layer)
19. [GRBL Firmware](#19-grbl-firmware)
20. [GRBL Configuration](#20-grbl-configuration)
21. [ESP32 Supervisory Controller](#21-esp32-supervisory-controller)
22. [UART Communication Architecture](#22-uart-communication-architecture)
23. [Encoder Feedback System](#23-encoder-feedback-system)
24. [TCA9548A I²C Multiplexer](#24-tca9548a-i²c-multiplexer)
25. [OLED System Monitor](#25-oled-system-monitor)
26. [Limit Switches and Homing](#26-limit-switches-and-homing)
27. [Motion Calibration](#27-motion-calibration)
28. [Motion Optimization](#28-motion-optimization)
29. [System Integration](#29-system-integration)
30. [Bring-Up Procedure](#30-bring-up-procedure)
31. [Engineering Challenges and Solutions](#31-engineering-challenges-and-solutions)
32. [Testing and Validation](#32-testing-and-validation)
33. [Performance Results](#33-performance-results)
34. [Computer-Vision Chess Architecture](#34-computer-vision-chess-architecture)
35. [Stockfish Integration](#35-stockfish-integration)
36. [ROS 2 Integration](#36-ros-2-integration)
37. [Future micro-ROS Architecture](#37-future-micro-ros-architecture)
38. [Known Limitations](#38-known-limitations)
39. [Future Improvements](#39-future-improvements)
40. [Project Status](#40-project-status)
41. [Author](#author)

---

# 1. Current System

> Add photographs of the completed IGOR V2 manipulator here.

```text
docs/images/final/front.jpg
docs/images/final/side.jpg
docs/images/final/rear.jpg
docs/images/final/electronics.jpg
docs/images/final/motion-test.jpg
```

IGOR V2 is a functional 3-DOF robotic manipulator integrating mechanical transmissions, stepper motors, distributed embedded controllers, position sensing, power electronics, and GRBL-based motion control.

### Core Features

* Custom 3-DOF robotic manipulator
* Three NEMA 17 stepper motors
* Custom 3D-printed mechanical structure
* Custom planetary gearbox
* GT2 and HTD3 belt transmissions
* Arduino Uno motion controller
* CNC Shield V3
* GRBL 1.1h firmware
* ESP32 supervisory controller
* UART communication between ESP32 and Arduino
* TMC2208 stepper driver
* TMC2209 stepper driver
* TMC2226 stepper driver
* 1/16 microstepping
* AS5600 magnetic encoders
* 12-bit absolute position sensing
* TCA9548A I²C multiplexer
* SH1106 1.3-inch OLED
* limit switches
* homing
* soft limits
* configurable acceleration
* configurable maximum velocity
* coordinated multi-axis movement
* 24 V power supply
* regulated motor and logic power rails
* emergency-stop system
* MG995 servo gripper
* custom CAD
* iterative 3D printing
* future computer-vision chess integration
* future Stockfish integration
* future ROS 2 and micro-ROS integration

---

# 2. Project Objectives

The primary objective was to develop a low-cost robotic manipulator capable of performing repeatable positioning tasks and eventually executing physical chess moves autonomously.

The design requirements were:

1. Design a custom 3-DOF robotic manipulator.
2. Use low-cost NEMA 17 stepper motors.
3. Increase joint torque using mechanical transmissions.
4. Develop custom 3D-printed mechanical components.
5. Implement multi-axis motion control.
6. Reduce vibration and improve motion smoothness.
7. Integrate absolute joint-position sensing.
8. Implement homing and limit switches.
9. Separate supervisory control from real-time motion generation.
10. Establish communication between multiple embedded controllers.
11. Provide local system monitoring.
12. Develop a modular architecture suitable for future expansion.
13. Support computer-vision integration.
14. Support autonomous chess gameplay.
15. Support future ROS 2 and micro-ROS integration.
16. Document the complete engineering development process.

---

# 3. System Specifications

| Parameter                   | Implementation                                            |
| --------------------------- | --------------------------------------------------------- |
| Robot Type                  | 3-DOF Serial Robotic Manipulator                          |
| Primary Application         | Autonomous Physical Chess Manipulation                    |
| Mechanical Design           | Custom CAD and 3D Printing                                |
| Main Supervisory Controller | ESP32                                                     |
| Motion Controller           | Arduino Uno                                               |
| Motion Firmware             | GRBL 1.1h                                                 |
| Motion Interface            | G-code                                                    |
| Motor Interface             | CNC Shield V3                                             |
| Joint Actuators             | 3 × NEMA 17 Stepper Motors                                |
| Motor Holding Torque        | Approximately 4.2 kg·cm                                   |
| Base Transmission           | 16T → 100T GT2 Belt Drive                                 |
| Shoulder Transmission       | 3.7:1 Planetary Gearbox + HTD3 Belt Transmission          |
| Elbow Transmission          | 16T → 100T GT2 Belt Drive                                 |
| Base Driver                 | TMC2208                                                   |
| Shoulder Driver             | TMC2209                                                   |
| Elbow Driver                | TMC2226                                                   |
| Microstepping               | 1/16                                                      |
| Position Sensors            | AS5600 Magnetic Encoders                                  |
| Encoder Resolution          | 12-bit / 4096 Positions                                   |
| I²C Expansion               | TCA9548A Multiplexer                                      |
| Local Display               | SH1106 1.3-inch OLED                                      |
| Controller Communication    | UART                                                      |
| Homing                      | GRBL Homing Cycle                                         |
| Safety Inputs               | Joint Limit Switches                                      |
| Power Supply                | 24 V, 5 A, 120 W                                          |
| Motor Rail                  | 18 V Regulated Supply                                     |
| Logic Rail                  | 5 V Regulated Supply                                      |
| Emergency Stop              | Mushroom Emergency-Stop Switch                            |
| End Effector                | MG995 Servo Gripper                                       |
| Robot Middleware            | ROS 2 — Integration Stage                                 |
| Future Embedded Middleware  | micro-ROS                                                 |
| Chess Engine                | Stockfish — Planned Integration                           |
| Computer Vision             | Chessboard-State Recognition Pipeline — Integration Stage |

---

# 4. System Architecture

```text
                         HIGH-LEVEL COMPUTING LAYER
                    PC / RASPBERRY PI / ROS 2 SYSTEM
                                   │
                                   │
                    COMPUTER VISION / STOCKFISH
                                   │
                                   ▼
                         HIGH-LEVEL COMMANDS
                                   │
                                   ▼
                    ┌───────────────────────────┐
                    │           ESP32           │
                    │                           │
                    │   Supervisory Controller  │
                    │                           │
                    │  • Encoder Acquisition    │
                    │  • System Monitoring      │
                    │  • OLED Interface         │
                    │  • Command Processing     │
                    │  • Future Kinematics      │
                    │  • Future micro-ROS       │
                    └─────────────┬─────────────┘
                                  │
                                  │ UART
                                  ▼
                    ┌───────────────────────────┐
                    │       ARDUINO UNO         │
                    │                           │
                    │         GRBL 1.1h         │
                    │                           │
                    │  • Step Generation        │
                    │  • Motion Planning        │
                    │  • Acceleration Profiles  │
                    │  • Homing                 │
                    │  • Limit Switches         │
                    │  • Soft Limits            │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                          CNC SHIELD V3
                                  │
                 ┌────────────────┼────────────────┐
                 │                │                │
                 ▼                ▼                ▼

             TMC2208          TMC2209          TMC2226

                 │                │                │
                 ▼                ▼                ▼

              BASE           SHOULDER           ELBOW

             NEMA 17          NEMA 17          NEMA 17


                    JOINT POSITION FEEDBACK SYSTEM

          BASE AS5600 ────────────────┐

      SHOULDER AS5600 ────────────────┼──► TCA9548A ───► ESP32

         ELBOW AS5600 ────────────────┘

                                             │
                                             ▼

                                         SH1106 OLED
```

---

# 5. IGOR V1

IGOR V1 was the first functional prototype of the robotic manipulator.

The objective of V1 was to validate:

* mechanical geometry
* joint actuation
* custom transmissions
* stepper-motor torque
* encoder integration
* basic motion control
* 3D-printed structural components
* end-effector integration

The system used NEMA 17 stepper motors and custom mechanical transmissions.

The shoulder joint incorporated a custom 3.7:1 planetary gearbox to increase available torque.

AS5600 magnetic encoders were integrated to measure joint position.

Motion commands were generated directly by the embedded firmware.

### V1 Achievements

* functional 3-axis mechanical structure
* custom planetary gearbox
* stepper-motor actuation
* magnetic encoder integration
* approximately 1.5 mm positioning precision during development testing
* functional MG995 servo gripper
* successful multi-axis motion testing
* proof of concept for the robotic platform

---

# 6. Why IGOR V2 Was Developed

The first-generation architecture demonstrated that the mechanical platform could operate successfully.

However, the original control system introduced several limitations.

These included:

* tightly coupled firmware
* limited motion buffering
* manual trajectory generation
* inconsistent acceleration profiles
* increased mechanical vibration
* limited motion scalability
* increasing firmware complexity
* difficulty integrating additional peripherals
* limited separation between high-level and low-level control

IGOR V2 redesigned the system around a distributed embedded architecture.

```text
IGOR V1

SINGLE EMBEDDED CONTROL SYSTEM

        │
        ├── SENSOR READING
        ├── MOTION GENERATION
        ├── MOTOR CONTROL
        ├── KINEMATICS
        └── SYSTEM MONITORING


                 ▼


IGOR V2

ESP32 SUPERVISORY CONTROLLER

        │
        │ UART
        ▼

ARDUINO UNO + GRBL

        │
        ▼

CNC SHIELD + TMC DRIVERS

        │
        ▼

STEPPER MOTORS
```

This architecture separates computational responsibilities and allows each controller to perform a more specialized role.

---

# 7. IGOR V2 Control Architecture

The V2 control system is divided into two embedded layers.

## High-Level Embedded Controller

```text
ESP32
```

Responsibilities:

* encoder acquisition
* I²C communication
* OLED monitoring
* system-state management
* high-level command handling
* UART communication
* future kinematic computation
* future ROS communication

## Motion-Control Layer

```text
ARDUINO UNO
     │
     ▼
GRBL 1.1h
     │
     ▼
CNC SHIELD V3
     │
     ▼
TMC DRIVERS
     │
     ▼
NEMA 17 MOTORS
```

Responsibilities:

* deterministic step generation
* coordinated motion
* acceleration control
* velocity control
* homing
* limit-switch handling
* soft limits
* command buffering
* G-code execution

This architecture improves modularity and simplifies future expansion.

---

# 8. Repository Structure

```text
IGOR/
│
├── README.md
├── LICENSE
│
├── cad/
│   ├── solidworks/
│   ├── step/
│   ├── stl/
│   ├── drawings/
│   └── renders/
│
├── hardware/
│   ├── bom/
│   │   └── BOM.csv
│   │
│   ├── electronics/
│   │   ├── system-architecture/
│   │   ├── wiring-diagrams/
│   │   └── power-distribution/
│   │
│   ├── motion-control/
│   │   ├── cnc-shield/
│   │   ├── driver-configuration/
│   │   └── limit-switches/
│   │
│   └── encoders/
│       ├── as5600/
│       └── tca9548a/
│
├── firmware/
│   ├── esp32/
│   │   ├── encoder-monitor/
│   │   ├── oled-interface/
│   │   ├── uart-bridge/
│   │   └── system-controller/
│   │
│   └── grbl/
│       ├── settings/
│       └── documentation/
│
├── software/
│   ├── computer-vision/
│   ├── chess-engine/
│   ├── kinematics/
│   ├── ros2/
│   └── micro-ros/
│
├── config/
│   ├── grbl/
│   │   └── final-settings.txt
│   └── ros2/
│
├── scripts/
│   ├── calibration/
│   ├── testing/
│   └── communication/
│
├── docs/
│   ├── build-log/
│   ├── architecture/
│   ├── troubleshooting/
│   ├── images/
│   │   ├── cad/
│   │   ├── mechanical/
│   │   ├── electronics/
│   │   ├── motion-tests/
│   │   ├── software/
│   │   └── final/
│   └── videos/
│
└── tests/
    ├── motion/
    ├── encoders/
    ├── homing/
    ├── repeatability/
    ├── communication/
    └── system/
```

---

# 9. Bill of Materials

| Category        | Component               |         Specification | Qty | Function                       |
| --------------- | ----------------------- | --------------------: | --: | ------------------------------ |
| Control         | ESP32 Development Board |            32-bit MCU |   1 | Supervisory controller         |
| Motion Control  | Arduino Uno             |            ATmega328P |   1 | GRBL controller                |
| Motor Interface | CNC Shield V3           |                3-axis |   1 | Stepper-driver interface       |
| Motor           | NEMA 17 Stepper         |            ~4.2 kg·cm |   3 | Joint actuation                |
| Driver          | TMC2208                 |    1/16 microstepping |   1 | Base motor                     |
| Driver          | TMC2209                 |    1/16 microstepping |   1 | Shoulder motor                 |
| Driver          | TMC2226                 |    1/16 microstepping |   1 | Elbow motor                    |
| Encoder         | AS5600                  |       12-bit magnetic |   3 | Joint position sensing         |
| I²C Expansion   | TCA9548A                | 8-channel multiplexer |   1 | Encoder bus expansion          |
| Display         | SH1106 OLED             |              1.3-inch |   1 | Local monitoring               |
| Power           | Switching PSU           |      24 V, 5 A, 120 W |   1 | Main supply                    |
| Regulation      | DC-DC Buck Converter    |           24 V → 18 V |   1 | Motor rail                     |
| Regulation      | DC-DC Buck Converter    |            24 V → 5 V |   1 | Logic rail                     |
| Safety          | Mushroom Emergency Stop |                   ___ |   1 | Main safety disconnect         |
| End Effector    | MG995 Servo             |     High-torque servo |   1 | Gripper actuation              |
| Transmission    | GT2 Belt and Pulleys    |            2 mm pitch | ___ | Base/elbow transmission        |
| Transmission    | HTD3 Belt and Pulleys   |            3 mm pitch | ___ | Shoulder transmission          |
| Gearbox         | Planetary Gearbox       |                 3.7:1 |   1 | Shoulder torque multiplication |
| Mechanical      | 3D-Printed Components   |                Custom | ___ | Robot structure                |
| Wiring          | Logic-Level Interface   | UART level adaptation |   1 | ESP32/Uno communication        |

---

# 10. Tools Required

## Electronics Tools

* soldering iron
* solder
* flux
* multimeter
* bench power supply
* wire cutters
* wire strippers
* heat-shrink tubing
* crimping tools
* oscilloscope recommended
* logic analyzer recommended

## Mechanical Tools

* 3D printer
* SolidWorks
* calipers
* hex keys
* screwdrivers
* files
* deburring tools
* bearing installation tools

## Software Tools

* Arduino IDE
* ESP-IDF or Arduino framework for ESP32
* GRBL 1.1h
* Universal G-code Sender or equivalent serial terminal
* Visual Studio Code
* Git
* SolidWorks
* ROS 2
* RViz
* MoveIt 2
* Python
* OpenCV
* Stockfish

---

# 11. Mechanical Design

IGOR was designed as a custom 3-axis serial robotic manipulator.

The complete mechanical structure was developed using SolidWorks.

The manipulator consists of:

* rotating base
* shoulder joint
* elbow joint
* forearm structure
* gripper end effector
* custom motor mounts
* bearing supports
* belt transmissions
* custom planetary gearbox
* encoder mounting structures
* electronics mounting system

The mechanical design was developed iteratively.

Important design constraints included:

* stepper-motor torque
* joint loading
* gearbox backlash
* belt tension
* pulley alignment
* bearing support
* structural deflection
* print orientation
* layer adhesion
* motor temperature
* encoder alignment
* cable routing
* serviceability

> Add complete CAD assembly render.

> Add exploded assembly.

> Add cross-section of shoulder gearbox.

> Add joint transmission diagrams.

---

# 12. Joint Transmission Architecture

## Base Joint

```text
NEMA 17 MOTOR

      │
      ▼

   16T GT2

      │
      ▼

   GT2 BELT

      │
      ▼

  100T GEAR

      │
      ▼

BASE ROTATION
```

Transmission ratio:

```text
16 / 100 = 0.16
```

The belt reduction increases available joint torque and improves positioning resolution.

---

## Shoulder Joint

```text
NEMA 17 MOTOR

      │
      ▼

3.7:1 PLANETARY GEARBOX

      │
      ▼

HTD3 BELT TRANSMISSION

      │
      ▼

SHOULDER JOINT
```

The shoulder joint experiences the highest gravitational load.

The custom planetary gearbox increases torque before the belt transmission stage.

The effective motion ratio used during calibration was approximately:

```text
0.054
```

---

## Elbow Joint

```text
NEMA 17 MOTOR

      │
      ▼

   16T GT2

      │
      ▼

   GT2 BELT

      │
      ▼

  100T GEAR

      │
      ▼

ELBOW ROTATION
```

Transmission ratio:

```text
16 / 100 = 0.16
```

---

# 13. Stepper Motors

All three joints use NEMA 17 stepper motors.

Approximate motor specification:

```text
Holding Torque: 4.2 kg·cm
```

Stepper motors were selected because they provide:

* simple position control
* high holding torque
* compatibility with GRBL
* wide driver availability
* low system cost
* repeatable incremental motion

Mechanical transmissions compensate for the limited direct-drive torque of the motors.

---

# 14. TMC Stepper Drivers

IGOR V2 uses three TMC-series stepper drivers.

| Axis     | Driver  |
| -------- | ------- |
| Base     | TMC2208 |
| Shoulder | TMC2209 |
| Elbow    | TMC2226 |

The TMC drivers replaced earlier stepper-driver configurations used during development.

The primary improvements were:

* smoother motor operation
* reduced audible noise
* reduced mechanical vibration
* improved low-speed movement
* configurable microstepping
* improved motion quality

Driver current limits must be configured according to the specific stepper motors and cooling conditions.

---

# 15. Microstepping

All joints operate at:

```text
1/16 MICROSTEPPING
```

Microstepping increases the number of commanded step positions per motor revolution.

The primary objective was not simply to increase theoretical positioning resolution.

The most important practical improvements were:

* smoother movement
* reduced vibration
* reduced resonance
* improved low-speed motion
* less abrupt joint movement

The final motion quality depends on:

* microstepping
* motor current
* acceleration
* maximum velocity
* mechanical stiffness
* gearbox backlash
* belt tension
* transmission ratio

---

# 16. Power-System Architecture

```text
                         230 V AC INPUT

                               │
                               ▼

                    24 V / 5 A / 120 W PSU

                               │
                               ▼

                    MUSHROOM EMERGENCY STOP

                               │
                  ┌────────────┴────────────┐
                  │                         │
                  ▼                         ▼

            MOTOR POWER PATH          LOGIC POWER PATH

                  │                         │
                  ▼                         ▼

          DC-DC BUCK CONVERTER       DC-DC BUCK CONVERTER

                  │                         │
                  ▼                         ▼

                18 V                       5 V

                  │                         │
                  ▼                         ├────────► ESP32
           CNC SHIELD V3                    │
                  │                         ├────────► ARDUINO UNO
                  │                         │
                  ▼                         ├────────► OLED
             TMC DRIVERS                    │
                  │                         └────────► ENCODER SYSTEM
                  ▼

            NEMA 17 MOTORS
```

---

# 17. Power Distribution

The system uses separate motor and logic power rails.

## Motor Rail

```text
24 V PSU

    │
    ▼

18 V BUCK CONVERTER

    │
    ▼

CNC SHIELD V3

    │
    ▼

STEPPER DRIVERS

    │
    ▼

NEMA 17 MOTORS
```

## Logic Rail

```text
24 V PSU

    │
    ▼

5 V BUCK CONVERTER

    │
    ├── ESP32
    ├── Arduino Uno
    ├── OLED
    └── Encoder Electronics
```

Separating motor and logic distribution improves system organization and simplifies power debugging.

---

# 18. Arduino Motion-Control Layer

The Arduino Uno acts as the dedicated motion-control processor.

It runs GRBL 1.1h.

The Arduino handles:

* step generation
* direction control
* acceleration profiles
* coordinated axis movement
* command buffering
* homing
* limit switches
* soft limits
* G-code interpretation

This allows the ESP32 to focus on supervisory tasks.

---

# 19. GRBL Firmware

GRBL provides the real-time motion-control layer.

```text
ESP32

   │
   ▼

G-CODE COMMAND

   │
   ▼

UART

   │
   ▼

ARDUINO UNO

   │
   ▼

GRBL MOTION PLANNER

   │
   ├── STEP GENERATION
   ├── ACCELERATION
   ├── VELOCITY CONTROL
   ├── COORDINATED MOVEMENT
   └── LIMIT HANDLING

   │
   ▼

CNC SHIELD

   │
   ▼

TMC DRIVERS

   │
   ▼

ROBOT JOINTS
```

The use of GRBL eliminated the need to manually implement a complete real-time multi-axis motion planner.

---

# 20. GRBL Configuration

The current development configuration is:

```text
$0=10
$1=25
$2=0
$3=4
$4=0
$5=1
$6=0

$10=1
$11=0.010
$12=0.002
$13=0

$20=1
$21=1
$22=1
$23=3

$24=80.000
$25=400.000
$26=250
$27=10.000

$30=1000
$31=0
$32=0

$100=3.470
$101=164.440
$102=55.560

$110=1500.000
$111=1000.000
$112=1000.000

$120=50.000
$121=50.000
$122=50.000

$130=295.000
$131=85.000
$132=45.000
```

Store the final validated configuration in:

```text
config/grbl/final-settings.txt
```

GRBL settings changed throughout development as:

* transmission ratios were calibrated
* axis directions were corrected
* homing directions were configured
* soft limits were implemented
* maximum velocities were tested
* acceleration values were tuned
* mechanical movement limits were measured

Only the final validated configuration should be treated as the production configuration.

---

# 21. ESP32 Supervisory Controller

The ESP32 acts as the high-level embedded controller.

Its responsibilities include:

* reading AS5600 encoders
* controlling the TCA9548A multiplexer
* updating the OLED display
* maintaining system state
* sending motion commands to GRBL
* receiving GRBL responses
* monitoring joint positions
* handling higher-level commands

Future responsibilities include:

* inverse kinematics
* trajectory command generation
* error monitoring
* ROS communication
* micro-ROS integration

---

# 22. UART Communication Architecture

The ESP32 communicates with the Arduino Uno through UART.

```text
ESP32

  TX ─────────────────────► RX

  RX ◄───────────────────── TX

  GND ───────────────────── GND

                │
                ▼

         ARDUINO UNO

                │
                ▼

             GRBL
```

Because the ESP32 uses 3.3 V logic while the Arduino Uno uses 5 V logic, voltage compatibility must be considered.

The communication interface was tested using logic-level conversion and resistor-divider approaches during development.

The final interface should be documented with:

* baud rate
* voltage levels
* level-shifting method
* grounding arrangement
* command format
* response parsing
* error handling

---

# 23. Encoder Feedback System

Each joint uses an AS5600 magnetic encoder.

The AS5600 provides:

```text
12-BIT ABSOLUTE ANGULAR POSITION

4096 POSITIONS PER REVOLUTION
```

The encoders provide joint-position information independently of stepper-motor command position.

```text
JOINT ROTATION

      │
      ▼

    MAGNET

      │
      ▼

   AS5600

      │
      ▼

TCA9548A MULTIPLEXER

      │
      ▼

     ESP32

      │
      ├── POSITION MONITORING
      ├── CALIBRATION
      ├── ERROR DETECTION
      └── FUTURE CLOSED-LOOP CONTROL
```

The current encoder system is primarily used for measurement and monitoring.

Future development can use encoder feedback for:

* missed-step detection
* motion verification
* joint error estimation
* automatic calibration
* closed-loop correction

---

# 24. TCA9548A I²C Multiplexer

All AS5600 sensors use the same default I²C address.

Multiple sensors therefore cannot be connected directly to the same I²C bus without address conflicts.

The TCA9548A solves this problem by creating multiple selectable I²C channels.

```text
ESP32 I²C BUS

      │
      ▼

   TCA9548A

      │
      ├── CHANNEL 0 ───► BASE AS5600
      │
      ├── CHANNEL 1 ───► SHOULDER AS5600
      │
      └── CHANNEL 2 ───► ELBOW AS5600
```

The ESP32 selects one multiplexer channel before communicating with each encoder.

---

# 25. OLED System Monitor

A 1.3-inch SH1106 OLED is connected to the ESP32.

The display is used for local system monitoring.

Displayed information can include:

* joint positions
* encoder values
* GRBL state
* communication status
* homing status
* error conditions
* system state

```text
ENCODER DATA

      │
      ▼

    ESP32

      │
      ▼

SH1106 OLED

      │
      ▼

LOCAL SYSTEM FEEDBACK
```

---

# 26. Limit Switches and Homing

IGOR uses physical limit switches with GRBL.

The switches provide:

* homing references
* travel-limit protection
* repeatable startup positions
* safer motion testing

The system uses a combination of normally-open and normally-closed configurations developed during prototype testing.

The final system should standardize the electrical configuration where possible.

The GRBL homing process is:

```text
SYSTEM START

      │
      ▼

ENABLE HOMING

      │
      ▼

SEARCH MOTION

      │
      ▼

LIMIT SWITCH ACTIVATED

      │
      ▼

PULL-OFF MOTION

      │
      ▼

SLOW LOCATE CYCLE

      │
      ▼

MACHINE COORDINATE ESTABLISHED
```

---

# 27. Motion Calibration

Each robot joint uses a different mechanical transmission ratio.

Therefore, each GRBL axis requires independent calibration.

The calibration process was:

```text
COMMAND ANGULAR MOVEMENT

        │
        ▼

MEASURE ACTUAL JOINT MOVEMENT

        │
        ▼

COMPARE COMMAND VS MEASUREMENT

        │
        ▼

CALCULATE CORRECTION FACTOR

        │
        ▼

UPDATE GRBL STEPS / UNIT

        │
        ▼

REPEAT TEST

        │
        ▼

VALIDATE MOTION
```

Calibration considered:

* motor step angle
* microstepping
* transmission ratio
* pulley ratio
* gearbox ratio
* mechanical backlash
* belt compliance
* encoder measurement

---

# 28. Motion Optimization

Motion quality improved significantly during V2 development.

The major changes were:

* migration to GRBL
* TMC-series drivers
* 1/16 microstepping
* improved acceleration control
* improved maximum-velocity tuning
* mechanical transmission adjustments
* belt-tension improvements
* distributed control architecture

The resulting system demonstrated:

* smoother movement
* reduced vibration
* improved low-speed control
* better acceleration behavior
* more predictable motion
* easier motion tuning
* improved multi-axis coordination

---

# 29. System Integration

IGOR combines multiple engineering disciplines.

```text
MECHANICAL DESIGN

        │
        ▼

TRANSMISSION DESIGN

        │
        ▼

MOTOR SELECTION

        │
        ▼

POWER ELECTRONICS

        │
        ▼

STEPPER DRIVERS

        │
        ▼

GRBL MOTION CONTROL

        │
        ▼

ESP32 SUPERVISORY CONTROL

        │
        ▼

ENCODER FEEDBACK

        │
        ▼

COMMUNICATION

        │
        ▼

COMPUTER VISION

        │
        ▼

CHESS ENGINE

        │
        ▼

ROS / micro-ROS
```

Examples of subsystem interaction included:

* transmission ratios affected GRBL calibration
* microstepping affected motion smoothness and maximum velocity
* acceleration settings affected structural vibration
* motor current affected torque and driver temperature
* encoder alignment affected position measurement
* power distribution affected controller stability
* UART voltage levels affected communication reliability
* mechanical backlash affected positioning accuracy
* cable routing affected joint movement
* firmware architecture affected future ROS integration

IGOR therefore evolved into a complete mechatronics and embedded-robotics platform rather than a standalone mechanical arm.

---

# 30. Bring-Up Procedure

Do not power and test the entire manipulator simultaneously during initial assembly.

A staged bring-up procedure makes faults easier to isolate.

## Stage 1 — Mechanical Assembly

Verify:

* free joint rotation
* bearing alignment
* pulley alignment
* gearbox operation
* belt tension
* fastener security

## Stage 2 — Power System

Validate:

```text
24 V PSU
    ↓
EMERGENCY STOP
    ↓
BUCK CONVERTERS
    ↓
18 V MOTOR RAIL
    ↓
5 V LOGIC RAIL
```

Measure all voltages before connecting controllers.

## Stage 3 — Single Motor Test

Test each axis independently.

Verify:

* direction
* driver current
* motor temperature
* smooth movement
* microstepping
* mechanical range

## Stage 4 — GRBL Motion

Test:

```text
G0 X10
G0 Y10
G0 Z10
```

Verify axis direction and travel.

## Stage 5 — Limit Switches

Verify each switch using the GRBL status report.

Test homing at low speed.

## Stage 6 — Encoders

Read each AS5600 independently.

Verify:

* correct multiplexer channel
* magnet alignment
* increasing/decreasing angle direction
* full measurement range
* repeatability

## Stage 7 — ESP32/Arduino Communication

Test:

* UART transmission
* GRBL command reception
* response parsing
* command acknowledgement
* error handling

## Stage 8 — Multi-Axis Motion

Test:

* coordinated movement
* acceleration
* vibration
* driver temperature
* motor temperature
* communication stability

## Stage 9 — Full System

Run:

* motion control
* encoders
* OLED
* UART
* homing
* limit switches
* end effector

Perform extended motion testing before integrating computer vision.

---

# 31. Engineering Challenges and Solutions

| Problem                               | Root Cause / Limitation                                     | Improvement                                                            |
| ------------------------------------- | ----------------------------------------------------------- | ---------------------------------------------------------------------- |
| High mechanical vibration             | Driver configuration, acceleration, transmission behavior   | Migrated to TMC drivers, microstepping, and GRBL acceleration profiles |
| Abrupt motor movement                 | Direct low-level motion generation                          | Implemented GRBL motion planning                                       |
| Increasing firmware complexity        | Single controller responsible for multiple subsystems       | Split architecture between ESP32 and Arduino                           |
| Limited torque at shoulder            | High gravitational joint loading                            | Added custom 3.7:1 planetary gearbox and belt reduction                |
| Encoder address conflicts             | All AS5600 sensors use the same I²C address                 | Added TCA9548A multiplexer                                             |
| UART voltage mismatch                 | ESP32 uses 3.3 V logic and Uno uses 5 V logic               | Tested level conversion and resistor-divider interface                 |
| Homing failures                       | Direction, switch logic, pull-off, and travel configuration | Tuned GRBL homing parameters                                           |
| Incorrect joint travel                | Transmission ratios and steps/unit calibration              | Per-axis calibration using measured joint movement                     |
| Stepper resonance                     | Motor speed, acceleration, and driver behavior              | Tuned acceleration, velocity, current, and microstepping               |
| Position uncertainty                  | Open-loop stepper control                                   | Added absolute magnetic encoder monitoring                             |
| Increasing system complexity          | Mechanical, sensing, control, and software subsystems       | Developed modular distributed architecture                             |
| Limited future middleware integration | GRBL controller not intended to run full robotics stack     | ESP32 supervisory layer and future micro-ROS architecture              |

---

# 32. Testing and Validation

A complete robotic system should be validated using measured data.

## Mechanical Tests

* joint range of motion
* gearbox backlash
* belt tension
* structural deflection
* gripper payload
* fastener loosening
* extended movement cycles

## Motion Tests

* minimum stable speed
* maximum stable speed
* acceleration
* vibration
* resonance
* coordinated motion
* repeated positioning

## Encoder Tests

* absolute angle accuracy
* repeatability
* magnet alignment
* zero-offset stability
* encoder noise
* missed-step detection

## Homing Tests

* repeated homing
* switch repeatability
* pull-off distance
* homing duration
* false triggers

## Electrical Tests

* motor-rail voltage
* logic-rail voltage
* driver temperature
* motor temperature
* buck-converter temperature
* controller stability

## Communication Tests

* UART command transmission
* command acknowledgement
* malformed commands
* communication recovery
* extended operation

## System Tests

* cold boot
* emergency stop
* homing
* repeated trajectories
* multi-axis movement
* end-effector operation
* extended-duration motion

---

# 33. Performance Results

Complete this section using final measured values.

| Metric                         | Result                                |
| ------------------------------ | ------------------------------------- |
| Number of Controlled Joints    | 3                                     |
| Base Range                     | ___°                                  |
| Shoulder Range                 | ___°                                  |
| Elbow Range                    | ___°                                  |
| Positioning Precision          | ~1.5 mm during V1 development testing |
| Base Maximum Velocity          | ___°/s                                |
| Shoulder Maximum Velocity      | ___°/s                                |
| Elbow Maximum Velocity         | ___°/s                                |
| Base Repeatability             | ___ mm / ___°                         |
| Shoulder Repeatability         | ___ mm / ___°                         |
| Elbow Repeatability            | ___ mm / ___°                         |
| Homing Repeatability           | ___°                                  |
| Maximum Payload                | ___ g                                 |
| Maximum Driver Temperature     | ___ °C                                |
| Maximum Motor Temperature      | ___ °C                                |
| UART Error Rate                | ___                                   |
| Longest Continuous Motion Test | ___ hours                             |

---

# 34. Computer-Vision Chess Architecture

The long-term application of IGOR is autonomous physical chess gameplay.

The computer-vision pipeline is designed to determine the state of a physical chessboard.

```text
CAMERA

   │
   ▼

IMAGE ACQUISITION

   │
   ▼

CHESSBOARD DETECTION

   │
   ▼

PERSPECTIVE CORRECTION

   │
   ▼

BOARD SEGMENTATION

   │
   ▼

64 INDIVIDUAL SQUARES

   │
   ▼

PIECE / OCCUPANCY DETECTION

   │
   ▼

CURRENT BOARD STATE

   │
   ▼

MOVE DETECTION

   │
   ▼

CHESS ENGINE
```

The vision subsystem is developed independently from the low-level motion controller.

This allows the motion-control architecture to remain modular.

---

# 35. Stockfish Integration

Stockfish is intended to provide the chess decision-making layer.

```text
PHYSICAL CHESSBOARD

        │
        ▼

COMPUTER VISION

        │
        ▼

BOARD STATE

        │
        ▼

FEN REPRESENTATION

        │
        ▼

STOCKFISH

        │
        ▼

BEST MOVE

        │
        ▼

CHESS COORDINATES

        │
        ▼

ROBOT TARGET POSITIONS

        │
        ▼

MOTION COMMANDS

        │
        ▼

IGOR
```

The high-level computer is responsible for:

* maintaining game state
* validating moves
* communicating with Stockfish
* converting chess moves into robot commands
* coordinating perception and manipulation

---

# 36. ROS 2 Integration

ROS 2 is used as the future high-level robotics middleware.

The robot model can be represented using:

* URDF
* SolidWorks-to-URDF exported meshes
* joint definitions
* link transforms
* robot-state publishing
* RViz visualization

The intended architecture is:

```text
ROS 2 COMPUTER

      │
      ├── COMPUTER VISION NODE
      ├── CHESS ENGINE NODE
      ├── ROBOT STATE
      ├── MOVEIT 2
      ├── KINEMATICS
      └── HIGH-LEVEL CONTROL

      │
      ▼

ESP32

      │
      ▼

GRBL MOTION CONTROLLER
```

ROS 2 integration remains separate from the validated embedded motion-control system.

---

# 37. Future micro-ROS Architecture

A future development objective is to integrate the ESP32 with micro-ROS.

```text
                    ROS 2 COMPUTING SYSTEM

                             │
                             ▼

                       micro-ROS AGENT

                             │
                             ▼

                            ESP32

                    micro-ROS CLIENT

                             │
            ┌────────────────┼────────────────┐
            │                │                │
            ▼                ▼                ▼

       JOINT STATE       COMMANDS        DIAGNOSTICS

            │
            ▼

          UART

            │
            ▼

      ARDUINO UNO

            │
            ▼

         GRBL

            │
            ▼

     CNC SHIELD V3

            │
            ▼

      ROBOT MOTION
```

Potential ROS interfaces include:

```text
/joint_states

/igor/joint_command

/igor/grbl_state

/igor/encoder_state

/igor/system_status
```

The objective is to maintain deterministic GRBL motion control while exposing the embedded robot controller to the ROS 2 ecosystem.

---

# 38. Known Limitations

* The current system primarily uses open-loop stepper motion control.
* AS5600 encoders are currently used mainly for monitoring rather than full closed-loop correction.
* Mechanical backlash affects positioning accuracy.
* 3D-printed structural components introduce compliance.
* The shoulder joint places significant load on the stepper motor and gearbox.
* GRBL operates in Cartesian machine-axis units rather than native robotic-joint semantics.
* Kinematic conversion must occur above the GRBL motion-control layer.
* ESP32-to-Arduino communication requires robust command and error handling.
* The current system uses multiple development boards instead of a custom integrated controller PCB.
* Computer vision, Stockfish, ROS 2, and micro-ROS integration remain development stages rather than validated final-system features.

---

# 39. Future Improvements

Potential future revisions include:

* complete encoder-based motion verification
* missed-step detection
* automatic encoder calibration
* closed-loop joint correction
* improved gearbox backlash
* stronger shoulder transmission
* improved cable management
* cable chains
* custom controller PCB
* integrated ESP32 and motor-control electronics
* isolated motor and logic power distribution
* current monitoring
* motor-temperature monitoring
* driver-temperature monitoring
* improved emergency-stop architecture
* joint-specific hardware limits
* improved homing repeatability
* trajectory generation
* inverse kinematics on ESP32
* forward kinematics validation
* workspace mapping
* payload characterization
* complete chessboard calibration
* robust chess-piece recognition
* automated move execution
* Stockfish integration
* ROS 2 hardware interface
* MoveIt 2 motion planning
* micro-ROS communication
* reproducible firmware build process
* automated installation scripts
* complete testing dataset
* quantitative repeatability analysis

---

# 40. Project Status

**IGOR V2 — Functional Embedded Robotic Manipulator Platform Under Active Development**

The current system successfully integrates:

* custom mechanical design
* 3D-printed robotic structure
* custom planetary gearbox
* belt-driven joint transmissions
* NEMA 17 stepper motors
* TMC stepper drivers
* 1/16 microstepping
* Arduino Uno
* CNC Shield V3
* GRBL motion control
* ESP32 supervisory control
* UART communication
* AS5600 magnetic encoders
* TCA9548A I²C multiplexing
* OLED system monitoring
* limit switches
* homing
* soft limits
* multi-axis motion
* configurable velocity
* configurable acceleration
* emergency-stop hardware
* end-effector actuation
* iterative mechanical and embedded-system debugging

The transition from IGOR V1 to IGOR V2 significantly changed the project architecture.

The first prototype demonstrated that a low-cost custom robotic manipulator could achieve functional multi-axis motion and position sensing.

The second generation focused on improving the quality and scalability of the control system.

Separating supervisory control from real-time motion generation, migrating to GRBL, integrating TMC-series stepper drivers, implementing microstepping, and maintaining independent encoder feedback created a more modular robotic platform.

The most important engineering outcome of IGOR is not simply the movement of a robotic arm.

It is the process of designing transmissions, developing embedded electronics, debugging communication interfaces, calibrating motion systems, integrating sensors, improving motion quality, and evolving the platform toward a distributed robotics architecture capable of supporting computer vision, autonomous chess gameplay, and ROS-based control.

---

# Author

**Nilin M**

Electronics and Communication Engineering

Embedded Systems • Robotics • Mechatronics • Motion Control • Embedded Linux
