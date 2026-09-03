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
│      Real Hardware (CTRE Phoenix 6)   │                      │           WPILib Simulation            │
│  • SwerveModuleIOKraken (TalonFX)     │                      │  • SwerveModuleIOSim (FlywheelSim)     │
│  • GyroIOPigeon2 (Pigeon2)            │                      │  • GyroIOSim (Kinematics Integration)  │
│  • ArmIOKraken (TalonFX)              │                      │  • ArmIOSim (SingleJointedArmSim)      │
│  • RollerIOKraken (TalonFX)           │                      │  • RollerIOSim (FlywheelSim)           │
│  • SequencerIOKraken (TalonFX)        │                      │  • SequencerIOSim (FlywheelSim)        │
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
│   ├── arm/                       # Arm pivot subsystem (ProfiledPID + Feedforward)
│   │   ├── Arm.java
│   │   ├── ArmIO.java
│   │   ├── ArmIOKraken.java
│   │   └── ArmIOSim.java
│   ├── roller/                    # Intake roller subsystem (Voltage control)
�            1. INTAKE                       2. SEQUENCE                         3. SHOOT
       ┌──────────────────┐            ┌──────────────────┐               ┌──────────────────┐
       │   Arm + Roller   │ ─────────> │    Sequencer     │ ────────────> │  Flywheel + Hood │
       │ (Sequential Move)│            │ (Spinning Feeder)│               │ (Auto-Aim Dynamic)│
       └──────────────────┘            └──────────────────┘               └──────────────────┘
```

### 1. Sequential Intake
* **Arm First**: The **Arm** pivots down to the ground angle ($-75^\circ$) while the roller and sequencer are stopped.
* **Roller Second**: Once `arm.atGoal()` is satisfied, the **Roller** wheels automatically spin forward at $+8\text{ V}$ to pull balls into the robot.
* **Auto-Retract**: Releasing the trigger immediately returns the arm to `STOW` ($0^\circ$) and stops the roller and sequencer.

### 2. Sequencing & Staging
* The **Sequencer** utilizes a high-traction spinning feeder motor controlled with closed-loop velocity feedforward ($kV = 0.12\text{ V/RPS}$, $kS = 0.25\text{ V}$, $kP = 0.1$).
* When ready to score, it accelerates to $50\text{ RPS}$ ($\approx 3000\text{ RPM}$) to feed all staged balls directly into the shooter flywheel.

### 3. Dynamic Auto-Aim & Shooting
When holding the **Right Trigger**:
1. **Target Tracking**: The robot calculates its distance $d = \sqrt{\Delta X^2 + \Delta Y^2}$ and required heading $\theta_{\text{target}} = \text{atan2}(\Delta Y, \Delta X) + 180^\circ$ relative to the active alliance goal.
2. **Heading Lock & Safety Speed Scaling**: The Swerve Drive automatically rotates the chassis to face the target ($\pm 2.0^\circ$ tolerance) while the driver continues to drive and translate freely with the left joystick (scaled to 70% max speed / 30% reduction for safety).
3. **Ballistics Interpolation**: Dynamically computes **Flywheel Speed** and **Hood Angle** from calibrated `InterpolatingDoubleTreeMap` curves.
4. **Auto-Feed**: Once the robot heading is locked, the flywheel is at speed ($\pm 2.5\text{ RPS}$), the hood is at angle ($\pm 1.0^\circ$), and the arm is at goal, the **Sequencer** spins to feed balls into the flywheel while the roller assists at $+8\text{ V}$!

---

## 🤖 Subsystems Breakdown

### 1. Swerve Drive
- **Module Geometry**: 4 independent modules with standard SDS MK4i configuration ($21.9\text{ in} \times 21.9\text{ in}$ track width & wheelbase).
- **Drive Motors**: Kraken X60 (TalonFX) with `VoltageOut(EnableFOC = true)` and current limits ($120\text{ A}$ stator / $60\text{ A}$ supply, $120\text{ A}$ slip limit).
- **Steer Motors**: Kraken X60 (TalonFX) with closed-loop onboard `PositionVoltage(EnableFOC = true)`, remote CANcoder feedback, and continuous wrap ($-0.5$ to $0.5$ rotations). Current limits: $60\text{ A}$ stator / $40\text{ A}$ supply.
- **Absolute Encoders**: CTRE CANcoder for absolute steering angle feedback.
- **IMU**: CTRE Pigeon 2 (`kPigeon2CanId = 20`).
- **Kinematics & Odometry**: `SwerveDriveKinematics` and `SwerveDriveOdometry` supporting field-relative driving, heading lock PID (`driveWithHeadingLock`), and speed desaturation.

### 2. Arm
- **Hardware**: NEO Vortex brushless motor with SPARK Flex controller (`kMotorId = 30`).
- **Control**: WPILib `ProfiledPIDController` + `ArmFeedforward` ($kP = 25.0$) with trapezoidal motion profiling ($v_{\max} = 2\pi\text{ rad/s}$, $a_{\max} = 4\pi\text{ rad/s}^2$) and gear ratio conversion factor ($50:1$).
- **Current Limits**: $60\text{ A}$ smart current limit, $12\text{ V}$ voltage compensation.
- **Role**: Positions the intake geometry and mechanism orientation ($-75^\circ$ ground intake, $+60^\circ$ shooting, $0^\circ$ stow).

### 3. Roller (Intake)
- **Hardware**: Dual Kraken X60 (TalonFX) (`kLeaderMotorId = 31`, `kFollowerMotorId = 32`).
- **Control**: Open-loop voltage control (`runIntake()` at $+8\text{ V}$, `runOuttake()` at $-8\text{ V}$, `runHold()` at $+2\text{ V}$, `stop()`).
- **Current Limits**: $60\text{ A}$ stator / $60\text{ A}$ supply.
- **Role**: Pulls balls into the robot from the floor or feeding station.

### 4. Sequencer (Feeder)
- **Hardware**: Dual NEO Vortex brushless motors with SPARK Flex controllers (`kLeaderMotorId = 33`, `kFollowerMotorId = 34`).
- **Control**: Leader-Follower configuration with velocity feedforward control using $kV = 0.12\text{ V/RPS}$, $kS = 0.25\text{ V}$, $kP = 0.1$. Setpoint: $50\text{ RPS}$ ($\approx 3000\text{ RPM}$).
- **Current Limits**: $60\text{ A}$ smart current limit, $12\text{ V}$ voltage compensation, brake mode.
- **Role**: Continuously spins forward to feed all staged balls directly into the shooter accelerator and flywheel.

### 5. Shooter (Flywheel + Accelerator + Adjustable Hood)
- **Hardware**:
  - **4-Motor Flywheel Array**: 4x Kraken X60 (TalonFX) (`kFlywheelLeaderMotorId = 35`, `kFlywheelFollower1MotorId = 36`, `kFlywheelFollower2MotorId = 38`, `kFlywheelFollower3MotorId = 39`) in leader-follower configuration. Current limits: $80\text{ A}$ stator / $60\text{ A}$ supply.
  - **Dual Accelerator / Kicker**: 2x NEO Vortex brushless motors with SPARK Flex controllers (`kAcceleratorLeaderMotorId = 40`, `kAcceleratorFollowerMotorId = 41`). Current limits: $60\text{ A}$ smart current limit.
  - **Adjustable Hood Motor**: 1x Kraken X60 (TalonFX) (`kHoodMotorId = 37`) for precision trajectory control ($0^\circ$ to $60^\circ$). Current limits: $40\text{ A}$ stator / $40\text{ A}$ supply.
- **Control**:
  - Flywheel: Closed-loop `VelocityVoltage(EnableFOC = true)` with Slot 0 PID ($kP = 0.15$) & Feedforward ($kV = 0.12$, $kS = 0.25$, $kA = 0.01$).
  - Accelerator: Closed-loop / feedforward velocity control ($60\text{ RPS}$, $\approx 3600\text{ RPM}$).
  - Hood: Closed-loop `PositionVoltage(EnableFOC = true)` with Slot 0 Position PID ($kP = 20.0$).
- **Role**: Accelerates balls to precise exit velocity while angling the hood for accurate target trajectory.

### 6. Superstructure State Machine
Coordinates all mechanisms into synchronized presets:

| State | Arm Angle | Roller Action | Shooter Action (Flywheel + Accelerator + Hood) | Sequencer Action |
|---|---|---|---|---|
| **`STOW`** | $0^\circ$ | `STOP` ($0\text{ V}$) | Flywheel `STOP`, Accelerator `STOP`, Hood $0^\circ$ | `STOP` ($0\text{ RPS}$) |
| **`INTAKE_GROUND`** | $-75^\circ$ | `INTAKE` ($+8\text{ V}$) | Flywheel `IDLE` ($20\text{ RPS}$), Accelerator `STOP`, Hood $0^\circ$ | `STOP` ($0\text{ RPS}$) |
| **`SPIN_UP_SHOOT`** | $+60^\circ$ | `HOLD` ($+2\text{ V}$) | Flywheel Spooling, Accelerator Spooling, Hood Positioning | `STOP` ($0\text{ RPS}$) |
| **`SHOOT`** | $+60^\circ$ | `INTAKE` ($+8\text{ V}$) | Flywheel At Speed, Accelerator At Speed, Hood At Angle | **`FEED`** ($50\text{ RPS}$) |

---

## 📡 CAN Bus & Hardware Map

| CAN ID | Device Type | Model | Subsystem / Location | Bus |
|---|---|---|---|---|
| **1** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Left Drive | CANivore / RIO |
| **2** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Left Steer | CANivore / RIO |
| **3** | Absolute Encoder | CANcoder | Swerve Front Left Angle | CANivore / RIO |
| **4** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Right Drive | CANivore / RIO |
| **5** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Right Steer | CANivore / RIO |
| **6** | Absolute Encoder | CANcoder | Swerve Front Right Angle | CANivore / RIO |
| **7** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Left Drive | CANivore / RIO |
| **8** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Left Steer | CANivore / RIO |
| **9** | Absolute Encoder | CANcoder | Swerve Back Left Angle | CANivore / RIO |
| **10** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Right Drive | CANivore / RIO |
| **11** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Right Steer | CANivore / RIO |
| **12** | Absolute Encoder | CANcoder | Swerve Back Right Angle | CANivore / RIO |
| **20** | IMU | Pigeon 2 | Drivetrain Heading | CANivore / RIO |
| **30** | Motor Controller | SPARK Flex (NEO Vortex) | Arm Pivot Motor | RIO CAN |
| **31** | Motor Controller | TalonFX (Kraken X60) | Intake Roller Leader Motor | CANivore / RIO |
| **32** | Motor Controller | TalonFX (Kraken X60) | Intake Roller Follower Motor | CANivore / RIO |
| **33** | Motor Controller | SPARK Flex (NEO Vortex) | Sequencer Feeder Leader Motor | RIO CAN |
| **34** | Motor Controller | SPARK Flex (NEO Vortex) | Sequencer Feeder Follower Motor | RIO CAN |
| **35** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Leader Motor | CANivore / RIO |
| **36** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Follower 1 Motor | CANivore / RIO |
| **37** | Motor Controller | TalonFX (Kraken X44/X60) | Shooter Adjustable Hood Motor | CANivore / RIO |
| **38** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Follower 2 Motor | CANivore / RIO |
| **39** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Follower 3 Motor | CANivore / RIO |
| **40** | Motor Controller | SPARK Flex (NEO Vortex) | Shooter Accelerator Leader Motor | RIO CAN |
| **41** | Motor Controller | SPARK Flex (NEO Vortex) | Shooter Accelerator Follower Motor | RIO CAN |

---

## ⚙️ Configuration & Tuning Guide

All constants are centralized in [`Constants.java`](src/main/java/frc/robot/Constants.java).

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
