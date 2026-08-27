# FRC Team 9062 — 2026 Off-Season Robot Code

WPILib Command-Based Java robot project for **Team 9062** built with **CTRE Phoenix 6**, **Kraken X60 (TalonFX)** brushless motors across all mechanisms, and an **AdvantageKit-style Decoupled I/O Architecture**.

---

## 📑 Table of Contents
1. [Architecture Overview](#-architecture-overview)
2. [Subsystems Breakdown](#-subsystems-breakdown)
   - [Swerve Drive](#1-swerve-drive)
   - [Sequencer](#2-sequencer)
   - [Arm](#3-arm)
   - [Superstructure State Machine](#4-superstructure-state-machine)
3. [CAN Bus & Hardware Map](#-can-bus--hardware-map)
4. [Configuration & Tuning Guide](#-configuration--tuning-guide)
5. [Controller Bindings](#-controller-bindings)
6. [Simulation vs Real Hardware](#-simulation-vs-real-hardware)
7. [Building & Deploying](#-building--deploying)

---

## 🚀 Architecture Overview

This project implements a **Decoupled I/O Architecture** (hardware abstraction layer) which separates high-level subsystem control logic from low-level motor and sensor hardware. This enables seamless physics simulation, deterministic unit testing, and straightforward hardware migration.

```
                               ┌────────────────────────────────────────────────────────┐
                               │                    RobotContainer                      │
                               └───────┬─────────────────┬───────────────────┬──────────┘
                                       │                 │                   │
                                       ▼                 ▼                   ▼
                               ┌───────────────┐ ┌───────────────┐ ┌────────────────────┐
                               │  SwerveDrive  │ │   Sequencer   │ │        Arm         │
                               └───────┬───────┘ └───────┬───────┘ └─────────┬──────────┘
                                       │                 │                   │
                                       │                 └─────────┬─────────┘
                                       │                           ▼
                                       │                 ┌───────────────────┐
                                       │                 │  Superstructure   │
                                       │                 └───────────────────┘
                                       ▼
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                          I/O Layer Interfaces                                          │
│                    SwerveModuleIO    │    GyroIO    │    SequencerIO    │    ArmIO                     │
└──────────────────────────────────────┴──────────────┴───────────────────┴──────────────────────────────┘
                    ▲                                                              ▲
                    │ (Hardware Mode - RobotBase.isReal())                         │ (Simulation Mode)
                    │                                                              │
┌───────────────────┴───────────────────┐                      ┌───────────────────┴────────────────────┐
│         Real Hardware (CANivore)      │                      │           WPILib Simulation            │
│  • SwerveModuleIOKraken (TalonFX)     │                      │  • SwerveModuleIOSim (FlywheelSim)     │
│  • GyroIOPigeon2 (Pigeon2)            │                      │  • GyroIOSim (Kinematics Integration)  │
│  • SequencerIOKraken (TalonFX)        │                      │  • SequencerIOSim (ElevatorSim)        │
│  • ArmIOKraken (TalonFX)              │                      │  • ArmIOSim (SingleJointedArmSim)      │
└───────────────────────────────────────┘                      └────────────────────────────────────────┘
```

---

## 🤖 Subsystems Breakdown

### 1. Swerve Drive
- **Module Geometry**: 4 independent modules with standard SDS MK4i L2 configuration.
- **Drive Motors**: Kraken X60 (TalonFX) controlled via `VoltageOut(EnableFOC = true)` with stator (80A) and supply (40A) current limits.
- **Steer Motors**: Kraken X60 (TalonFX) with closed-loop onboard `PositionVoltage(EnableFOC = true)`, remote CANcoder feedback, and continuous wrap ($-0.5$ to $0.5$ rotations).
- **Absolute Encoders**: CTRE CANcoder for absolute steering angle feedback.
- **IMU**: CTRE Pigeon 2 on CANivore.
- **Kinematics & Odometry**: `SwerveDriveKinematics` and `SwerveDriveOdometry` supporting field-relative and robot-relative driving, cosine/angle optimization, and speed desaturation.

### 2. Sequencer
- **Hardware**: Kraken X60 (TalonFX) on CANivore.
- **Control**: WPILib `ProfiledPIDController` + `ElevatorFeedforward` with trapezoidal motion profiling and gravity compensation.
- **Auto-Zeroing**: Auto-zeros position when the lower limit switch is tripped.
- **Simulation**: Modeled with WPILib `ElevatorSim` with gravity compensation and 2-motor Kraken gearbox model.

### 3. Arm
- **Hardware**: Kraken X60 (TalonFX) on CANivore.
- **Control**: WPILib `ProfiledPIDController` + `ArmFeedforward` with cosine gravity compensation and trapezoidal motion profiling.
- **Simulation**: Modeled with WPILib `SingleJointedArmSim` with gravity compensation and moment of inertia calculation.

### 4. Superstructure State Machine
Coordinates the synchronized positioning of the Sequencer and Arm into predefined named setpoints:
- `STOW`: Safe home travel position ($0.0\text{ m}, 0^\circ$).
- `INTAKE_GROUND`: Floor intake deployment ($0.10\text{ m}, -45^\circ$).
- `INTAKE_SOURCE`: Feeder station intake ($0.60\text{ m}, 30^\circ$).
- `SCORE_LOW`: Low scoring location ($0.30\text{ m}, 45^\circ$).
- `SCORE_MID`: Mid scoring location ($0.65\text{ m}, 60^\circ$).
- `SCORE_HIGH`: High scoring location ($1.10\text{ m}, 75^\circ$).
- `CLIMB`: Climb ready configuration ($1.20\text{ m}, 0^\circ$).

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
| **10** *(Default)* | Motor Controller | TalonFX (Kraken X60) | Swerve Back Right Drive |
| **11** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Right Steer |
| **12** | Absolute Encoder | CANcoder | Swerve Back Right Angle |
| **20** | Motor Controller | TalonFX (Kraken X60) | Sequencer Lift Motor |

*(Note: Adjust any overlapping IDs in `Constants.java` to match your physical robot wiring.)*

---

## ⚙️ Configuration & Tuning Guide

All constants are centralized in [`Constants.java`](src/main/java/frc/robot/Constants.java). Items requiring calibration on your physical robot are explicitly tagged with `// TODO: Tune this value`.

### 1. Swerve Module Offsets & Robot Dimensions
- `kTrackWidthMeters`: Center-to-center distance between left and right wheels.
- `kWheelbaseMeters`: Center-to-center distance between front and back wheels.
- `kWheelRadiusMeters`: Wheel radius in meters (e.g. $2\text{ in} = 0.0508\text{ m}$).
- CANcoder offsets in rotations:
  - `kFrontLeftCANcoderOffsetRotations`
  - `kFrontRightCANcoderOffsetRotations`
  - `kBackLeftCANcoderOffsetRotations`
  - `kBackRightCANcoderOffsetRotations`

### 2. Control Gains (PID + Feedforward)
- **Arm**:
  - `kProportionalGain`, `kIntegralGain`, `kDerivativeGain`
  - `kStaticGain` ($kS$), `kGravityGain` ($kG$), `kVelocityGain` ($kV$), `kAccelerationGain` ($kA$)
  - `kMaxVelocityRadiansPerSecond`, `kMaxAccelerationRadiansPerSecondSquared`
- **Sequencer**:
  - `kProportionalGain`, `kIntegralGain`, `kDerivativeGain`
  - `kStaticGain` ($kS$), `kGravityGain` ($kG$), `kVelocityGain` ($kV$), `kAccelerationGain` ($kA$)
  - `kMaxVelocityMetersPerSecond`, `kMaxAccelerationMetersPerSecondSquared`
- **Swerve Drive**:
  - Drive: `kDriveProportionalGain`, `kDriveVelocityGain`
  - Steer: `kSteerProportionalGain`, `kSteerDerivativeGain`

### 3. Current Limits
- Configured with stator current limits (peak torque) and supply current limits (battery protection):
  - **Swerve Drive**: 80A stator / 40A supply
  - **Swerve Steer**: 40A stator / 20A supply
  - **Sequencer**: 80A stator / 40A supply
  - **Arm**: 60A stator / 40A supply

---

## 🎮 Controller Bindings

### Driver Controller (Port 0 - Xbox Controller)
| Input | Action |
|---|---|
| **Left Stick Y (Inverted)** | Translate Forward / Backward (Field-Relative) |
| **Left Stick X (Inverted)** | Translate Left / Right (Field-Relative) |
| **Right Stick X** | Rotate Left / Right |
| **Start Button** | Reset Gyro Heading to 0° |

### Operator Controller (Port 1 - Xbox Controller)
| Input | Action |
|---|---|
| **A (Hold)** | Superstructure → `INTAKE_GROUND` |
| **B (Hold)** | Superstructure → `INTAKE_SOURCE` |
| **X (Hold)** | Superstructure → `SCORE_LOW` |
| **Y (Hold)** | Superstructure → `SCORE_MID` |
| **Right Bumper (Hold)** | Superstructure → `SCORE_HIGH` |
| **Left Bumper (Hold)** | Superstructure → `CLIMB` |
| *(Default / Released)* | Superstructure → `STOW` |

---

## 🔄 Simulation vs Real Hardware

In [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java), hardware selection is automatic:

```java
if (RobotBase.isReal()) {
  // Instantiates real Kraken X60 motors, CANcoders, and Pigeon 2 on CANivore
  m_sequencer = new Sequencer(new SequencerIOKraken(SequencerConstants.kMotorId));
  m_arm = new Arm(new ArmIOKraken(ArmConstants.kMotorId));
  m_swerveDrive = new SwerveDrive(new GyroIOPigeon2(...), new SwerveModuleIOKraken(...), ...);
} else {
  // Instantiates WPILib physics simulation (ElevatorSim, SingleJointedArmSim, FlywheelSim)
  m_sequencer = new Sequencer(new SequencerIOSim());
  m_arm = new Arm(new ArmIOSim());
  m_swerveDrive = new SwerveDrive(new GyroIOSim(), new SwerveModuleIOSim(), ...);
}
```

---

## 🛠️ Building & Deploying

### Build Project
```bash
./gradlew build
```

### Run Simulation
```bash
./gradlew simulateJava
```

### Deploy to Robot (via USB / Ethernet / Radio)
```bash
./gradlew deploy
```
