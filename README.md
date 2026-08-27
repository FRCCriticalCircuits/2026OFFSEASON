# FRC Team 9062 — 2026 Off-Season Robot Code

WPILib Command-Based Java robot project for **Team 9062** built with **CTRE Phoenix 6**, **Kraken X60 (TalonFX)** brushless motors across all mechanisms, and an **AdvantageKit-style Decoupled I/O Architecture** featuring an **Automated Ball Pipeline with Dynamic Auto-Aim**.

---

## 📑 Table of Contents
1. [Architecture Overview](#-architecture-overview)
2. [Project File Structure](#-project-file-structure)
3. [Ball Pipeline & Auto-Aim System](#-ball-pipeline--auto-aim-system)
   - [Sequential Intake](#1-sequential-intake)
   - [Sequencing & Staging](#2-sequencing--staging)
   - [Dynamic Auto-Aim & Shooting](#3-dynamic-auto-aim--shooting)
4. [Subsystems Breakdown](#-subsystems-breakdown)
   - [Swerve Drive](#1-swerve-drive)
   - [Arm](#2-arm)
   - [Roller (Intake)](#3-roller-intake)
   - [Sequencer](#4-sequencer)
   - [Shooter (Flywheel + Adjustable Hood)](#5-shooter-flywheel--adjustable-hood)
   - [Superstructure State Machine](#6-superstructure-state-machine)
5. [CAN Bus & Hardware Map](#-can-bus--hardware-map)
6. [Configuration & Tuning Guide](#-configuration--tuning-guide)
7. [Controller Bindings](#-controller-bindings)
8. [Telemetry & SmartDashboard](#-telemetry--smartdashboard)
9. [Simulation vs Real Hardware](#-simulation-vs-real-hardware)
10. [Building & Deploying](#-building--deploying)

---

## 🚀 Architecture Overview

This project implements a **Decoupled I/O Architecture** (hardware abstraction layer) which separates high-level subsystem control logic from low-level motor and sensor hardware. This enables seamless physics simulation, deterministic unit testing, and straightforward hardware migration.

```
                               ┌────────────────────────────────────────────────────────┐
                               │                    RobotContainer                      │
                               └───────┬─────────┬─────────┬─────────┬─────────┬────────┘
                                       │         │         │         │         │
                                       ▼         ▼         ▼         ▼         ▼
                                  SwerveDrive   Arm     Roller   Sequencer  Shooter
                                       │         │         │         │   (Flywheel+Hood)
                                       │         └─────────┼─────────┴─────────┘
                                       │                   ▼
                                       │         ┌───────────────────┐
                                       │         │  Superstructure   │
                                       │         └───────────────────┘
                                       ▼
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                          I/O Layer Interfaces                                          │
│           SwerveModuleIO    │    GyroIO    │    ArmIO    │   RollerIO   │   SequencerIO   │  ShooterIO │
└─────────────────────────────┴──────────────┴─────────────┴──────────────┴─────────────────┴────────────┘
                    ▲                                                              ▲
                    │ (Hardware Mode - RobotBase.isReal())                         │ (Simulation Mode)
                    │                                                              │
┌───────────────────┴───────────────────┐                      ┌───────────────────┴────────────────────┐
│         Real Hardware (CANivore)      │                      │           WPILib Simulation            │
│  • SwerveModuleIOKraken (TalonFX)     │                      │  • SwerveModuleIOSim (FlywheelSim)     │
│  • GyroIOPigeon2 (Pigeon2)            │                      │  • GyroIOSim (Kinematics Integration)  │
│  • ArmIOKraken (TalonFX)              │                      │  • ArmIOSim (SingleJointedArmSim)      │
│  • RollerIOKraken (TalonFX)           │                      │  • RollerIOSim (FlywheelSim)           │
│  • SequencerIOKraken (TalonFX)        │                      │  • SequencerIOSim (ElevatorSim)        │
│  • ShooterIOKraken (3x TalonFX)       │                      │  • ShooterIOSim (Flywheel+Hood Sim)    │
└───────────────────────────────────────┘                      └────────────────────────────────────────┘
```

---

## 📁 Project File Structure

```
src/main/java/frc/robot/
├── Constants.java                 # Central constants, CAN IDs, current limits & PID gains
├── Main.java                      # Robot program entry point
├── Robot.java                     # TimedRobot lifecycle & CommandScheduler executor
├── RobotContainer.java            # Subsystem instantiation, hardware/sim routing & button bindings
├── subsystems/
│   ├── arm/                       # Arm pivot subsystem
│   │   ├── Arm.java
│   │   ├── ArmIO.java
│   │   ├── ArmIOKraken.java
│   │   └── ArmIOSim.java
│   ├── roller/                    # Intake roller subsystem
│   │   ├── Roller.java
│   │   ├── RollerIO.java
│   │   ├── RollerIOKraken.java
│   │   └── RollerIOSim.java
│   ├── sequencer/                 # Ball indexer / elevator carriage subsystem
│   │   ├── Sequencer.java
│   │   ├── SequencerIO.java
│   │   ├── SequencerIOKraken.java
│   │   └── SequencerIOSim.java
│   ├── shooter/                   # Combined Flywheel + Hood shooter subsystem
│   │   ├── Shooter.java
│   │   ├── ShooterIO.java
│   │   ├── ShooterIOKraken.java
│   │   └── ShooterIOSim.java
│   └── swerve/                    # 4-module swerve drivetrain subsystem
│       ├── GyroIO.java
│       ├── GyroIOPigeon2.java
│       ├── GyroIOSim.java
│       ├── SwerveDrive.java
│       ├── SwerveModuleIO.java
│       ├── SwerveModuleIOKraken.java
│       └── SwerveModuleIOSim.java
├── superstructure/                # High-level state coordinator
│   ├── Superstructure.java
│   └── SuperstructureState.java
└── util/
    └── AutoAim.java               # Ballistics curves & real-time target distance calculations
```

---

## 🎯 Ball Pipeline & Auto-Aim System

```
 ┌──────────────────────────────────────────────────────────────────────────────────────────────┐
 │                                   BALL MANAGEMENT PIPELINE                                   │
 └──────────────────────────────────────────────────────────────────────────────────────────────┘
            1. INTAKE                       2. SEQUENCE                         3. SHOOT
       ┌──────────────────┐            ┌──────────────────┐               ┌──────────────────┐
       │   Arm + Roller   │ ─────────> │    Sequencer     │ ────────────> │  Flywheel + Hood │
       │ (Sequential Move)│            │ (Index / Feed)   │               │ (Auto-Aim Dynamic)│
       └──────────────────┘            └──────────────────┘               └──────────────────┘
```

### 1. Sequential Intake
* **Arm First**: The **Arm** pivots down to the ground angle ($-45^\circ$) while the roller is stopped.
* **Roller Second**: Once `arm.atGoal()` is satisfied, the **Roller** wheels automatically spin forward at $+10\text{ V}$ to pull balls into the robot.
* **Auto-Retract**: Releasing the trigger immediately returns the arm to `STOW` ($0^\circ$) and stops the roller.

### 2. Sequencing & Staging
* The **Sequencer** indexes balls from the intake and lifts/stages them inside the robot.
* Automatically zeros its position whenever the lower limit switch is triggered.

### 3. Dynamic Auto-Aim & Shooting
When holding the **Right Trigger**:
1. **Target Tracking**: The robot calculates its distance $d = \sqrt{\Delta X^2 + \Delta Y^2}$ and required heading $\theta_{\text{target}} = \text{atan2}(\Delta Y, \Delta X)$ relative to the alliance goal.
2. **Heading Lock & Safety Speed Scaling**: The Swerve Drive automatically rotates the chassis to face the target while the driver continues to drive and translate freely with the left joystick (scaled to 70% max speed / 30% reduction for safety).
3. **Ballistics Interpolation**: Dynamically computes **Flywheel Speed** and **Hood Angle** from calibrated `InterpolatingDoubleTreeMap` curves.
4. **Auto-Feed**: Once the robot heading is locked ($\pm 1.5^\circ$), the flywheel is at speed, and the hood is at the angle, the **Sequencer** feeds balls into the flywheel!

---

## 🤖 Subsystems Breakdown

### 1. Swerve Drive
- **Module Geometry**: 4 independent modules with standard SDS MK4i L2 configuration.
- **Drive Motors**: Kraken X60 (TalonFX) with `VoltageOut(EnableFOC = true)` and current limits (80A stator / 40A supply).
- **Steer Motors**: Kraken X60 (TalonFX) with closed-loop onboard `PositionVoltage(EnableFOC = true)`, remote CANcoder feedback, and continuous wrap ($-0.5$ to $0.5$ rotations).
- **Absolute Encoders**: CTRE CANcoder for absolute steering angle feedback.
- **IMU**: CTRE Pigeon 2 on CANivore.
- **Kinematics & Odometry**: `SwerveDriveKinematics` and `SwerveDriveOdometry` supporting field-relative driving, heading lock PID (`driveWithHeadingLock`), and speed desaturation.

### 2. Arm
- **Hardware**: Kraken X60 (TalonFX) on CANivore.
- **Control**: WPILib `ProfiledPIDController` + `ArmFeedforward` with cosine gravity compensation and trapezoidal motion profiling.
- **Role**: Positions the intake geometry and mechanism orientation.

### 3. Roller (Intake)
- **Hardware**: Kraken X60 (TalonFX) on CANivore.
- **Control**: Open-loop voltage control (`runIntake()`, `runOuttake()`, `runHold()`, `stop()`).
- **Role**: Pulls balls into the robot from the floor or feeding station.

### 4. Sequencer
- **Hardware**: Kraken X60 (TalonFX) on CANivore.
- **Control**: WPILib `ProfiledPIDController` + `ElevatorFeedforward` with trapezoidal motion profiling and gravity compensation.
- **Auto-Zeroing**: Auto-zeros position when the lower limit switch is tripped.
- **Role**: Indexes and feeds balls from intake to shooter.

### 5. Shooter (Flywheel + Adjustable Hood)
- **Hardware**:
  - **Dual Flywheel Motors**: 2x Kraken X60 (TalonFX) on CANivore in leader-follower configuration (`MotorAlignmentValue.Opposed`).
  - **Adjustable Hood Motor**: 1x Kraken X60 (TalonFX) on CANivore for precision trajectory control.
- **Control**:
  - Flywheel: Closed-loop `VelocityVoltage(EnableFOC = true)` with Slot 0 PID & Feedforward.
  - Hood: Closed-loop `PositionVoltage(EnableFOC = true)` with Slot 0 Position PID.
- **Role**: Accelerates balls to precise exit velocity while angling the hood for accurate target trajectory.

### 6. Superstructure State Machine
Coordinates all mechanisms into synchronized presets:

| State | Sequencer Height | Arm Angle | Roller Action | Shooter Action (Flywheel + Hood) |
|---|---|---|---|---|
| **`STOW`** | $0.00\text{ m}$ | $0^\circ$ | `STOP` | Flywheel `STOP`, Hood $0^\circ$ |
| **`INTAKE_GROUND`** | $0.10\text{ m}$ | $-45^\circ$ | `INTAKE` | Flywheel `IDLE`, Hood $0^\circ$ |
| **`SPIN_UP_SHOOT`** | $0.80\text{ m}$ | $+60^\circ$ | `HOLD` | Flywheel Spooling, Hood Positioning |
| **`SHOOT`** | $0.80\text{ m}$ | $+60^\circ$ | `INTAKE` (feed) | Flywheel At Speed, Hood At Angle |

---

## 📡 CAN Bus & Hardware Map

All CTRE devices reside on the high-speed **CANivore** bus (`"canivore"`).

| CAN ID | Device Type | Model | Subsystem / Location |
|---|---|---|---|
| **0** | IMU | Pigeon 2 | Drivetrain |
| **1** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Left Drive |
| **2** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Left Steer |
| **3** | Absolute Encoder | CANcoder | Swerve Front Left Angle |
| **4** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Right Drive |
| **5** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Right Steer |
| **6** | Absolute Encoder | CANcoder | Swerve Front Right Angle |
| **7** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Left Drive |
| **8** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Left Steer |
| **9** | Absolute Encoder | CANcoder | Swerve Back Left Angle |
| **10** | Motor Controller | TalonFX (Kraken X60) | Arm Pivot Motor |
| **11** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Right Steer |
| **12** | Absolute Encoder | CANcoder | Swerve Back Right Angle |
| **13** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Right Drive |
| **20** | Motor Controller | TalonFX (Kraken X60) | Sequencer Lift Motor |
| **30** | Motor Controller | TalonFX (Kraken X60) | Intake Roller Motor |
| **40** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Leader |
| **41** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Follower |
| **42** | Motor Controller | TalonFX (Kraken X60) | Shooter Adjustable Hood Motor |

---

## ⚙️ Configuration & Tuning Guide

All constants are centralized in [`Constants.java`](src/main/java/frc/robot/Constants.java). Items requiring calibration on your physical robot are explicitly tagged with `// TODO: Tune this value`.

> 📋 **Detailed Tuning Checklist**: See [`TODO.md`](TODO.md) for a full step-by-step checklist, measurement procedures, and tuning instructions for every variable.

### 1. Swerve Module Offsets & Robot Dimensions
- `kTrackWidthMeters`: Center-to-center distance between left and right wheels.
- `kWheelbaseMeters`: Center-to-center distance between front and back wheels.
- `kWheelRadiusMeters`: Wheel radius in meters (e.g. $2\text{ in} = 0.0508\text{ m}$).
- CANcoder offsets: `kFrontLeftCANcoderOffsetRotations`, `kFrontRightCANcoderOffsetRotations`, etc.

### 2. Auto-Aim & Ballistics Calibration
- `kBlueGoalLocation` & `kRedGoalLocation`: Goal coordinate offsets in field space.
- `kHeadingProportionalGain`: Swerve heading lock rotation strength.
- `AutoAim.m_flywheelSpeedMap` & `AutoAim.m_hoodAngleMap`: Empirical distance-to-speed/angle tables.

### 3. Current Limits
- **Swerve Drive**: 80A stator / 40A supply
- **Swerve Steer**: 40A stator / 20A supply
- **Sequencer**: 80A stator / 40A supply
- **Shooter Flywheel**: 80A stator / 40A supply
- **Shooter Hood**: 40A stator / 20A supply
- **Arm**: 60A stator / 40A supply
- **Roller**: 60A stator / 40A supply

---

## 🎮 Controller Bindings

### Single Driver Controller (Port 0 - Xbox Controller)
| Input | Subsystem / Mechanism Action |
|---|---|
| **Left Stick Y (Inverted)** | Translate Forward / Backward (Field-Relative) |
| **Left Stick X (Inverted)** | Translate Left / Right (Field-Relative) |
| **Right Stick X** | Rotate Left / Right |
| **Start Button** | Reset Gyro Heading to 0° |
| **Left Trigger (Hold)** | **SEQUENTIAL GROUND INTAKE**: Arm deploys $\rightarrow$ waits for angle $\rightarrow$ spins roller $\rightarrow$ stows on release |
| **Right Trigger (Hold)** | **DYNAMIC AUTO-AIM & SHOOT**: Rotates chassis to goal + sets flywheel speed & hood angle from distance $\rightarrow$ auto-feeds |
| *(Released / Default)* | Superstructure → `STOW` (Automatic home position) |

---

## 📊 Telemetry & SmartDashboard

The following telemetry values are published every 20ms periodic cycle:
- **Drivetrain**: `Swerve/Pose X Meters`, `Swerve/Pose Y Meters`, `Swerve/Heading Degrees`, `Swerve/Module {0-3}/Speed (m per sec)`, `Swerve/Module {0-3}/Angle (deg)`
- **Auto-Aim**: `AutoAim/Target Distance (m)`, `AutoAim/Target Heading (deg)`, `AutoAim/Heading Aligned`, `AutoAim/Ready To Fire`
- **Arm**: `Arm/Angle (deg)`, `Arm/Velocity (deg per sec)`, `Arm/Goal (deg)`, `Arm/Applied Output (V)`, `Arm/Current (A)`, `Arm/At Goal`
- **Sequencer**: `Sequencer/Height (m)`, `Sequencer/Velocity (m per sec)`, `Sequencer/Goal (m)`, `Sequencer/Applied Output (V)`, `Sequencer/Current (A)`, `Sequencer/At Goal`
- **Roller**: `Roller/Velocity (RPS)`, `Roller/Applied Output (V)`, `Roller/Current (A)`, `Roller/Game Piece Detected`
- **Shooter**: `Shooter/Flywheel Velocity (RPS)`, `Shooter/Flywheel Target (RPS)`, `Shooter/Hood Angle (deg)`, `Shooter/Hood Target (deg)`, `Shooter/Ready To Shoot`
- **Superstructure**: `Superstructure/Current State`, `Superstructure/Desired State`, `Superstructure/At Goal`

---

## 🔄 Simulation vs Real Hardware

In [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java), hardware selection is automatic:

```java
if (RobotBase.isReal()) {
  // Instantiates real Kraken X60 motors, CANcoders, and Pigeon 2 on CANivore
  m_sequencer = new Sequencer(new SequencerIOKraken(...));
  m_arm = new Arm(new ArmIOKraken(...));
  m_roller = new Roller(new RollerIOKraken(...));
  m_shooter = new Shooter(new ShooterIOKraken(...));
  m_swerveDrive = new SwerveDrive(new GyroIOPigeon2(...), new SwerveModuleIOKraken(...), ...);
} else {
  // Instantiates WPILib physics simulations (ElevatorSim, SingleJointedArmSim, FlywheelSim)
  m_sequencer = new Sequencer(new SequencerIOSim());
  m_arm = new Arm(new ArmIOSim());
  m_roller = new Roller(new RollerIOSim());
  m_shooter = new Shooter(new ShooterIOSim());
  m_swerveDrive = new SwerveDrive(new GyroIOSim(), new SwerveModuleIOSim(), ...);
}
```

---

## 🛠️ Building & Deploying

### Build Project
```bash
./gradlew build
```

### Run Unit Tests
```bash
./gradlew test
```

### Run Desktop Simulation
```bash
./gradlew simulateJava
```

### Deploy to Robot (via USB / Ethernet / Radio)
```bash
./gradlew deploy
```
