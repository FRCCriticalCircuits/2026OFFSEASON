# FRC Team 9062 — 2026 Off-Season Robot Code

WPILib Command-Based Java robot project for **Team 9062** built with **CTRE Phoenix 6**, **REV SPARK MAX**, an **AdvantageKit-style Decoupled I/O Architecture**, and flexible **Autonomous and Teleoperated Ball Handling Mechanisms**.

---

## 📑 Table of Contents
1. [Architecture Overview](#-architecture-overview)
2. [Project File Structure](#-project-file-structure)
3. [Autonomous Mode](#-autonomous-mode)
4. [Ball Pipeline & Control Modes](#-ball-pipeline--control-modes)
   - [Active Teleop Manual Controls](#1-active-teleop-manual-controls)
   - [Sequential Intake & Dynamic Auto-Aim (Automated Pipeline)](#2-sequential-intake--dynamic-auto-aim-automated-pipeline)
5. [Subsystems Breakdown](#-subsystems-breakdown)
   - [Swerve Drive](#1-swerve-drive)
   - [Arm Pivot](#2-arm-pivot)
   - [Intake Roller](#3-intake-roller)
   - [Sequencer Feeder](#4-sequencer-feeder)
   - [Shooter (Flywheel + Hood + Supporting Shooter)](#5-shooter-flywheel--hood--supporting-shooter)
   - [Superstructure Coordinator](#6-superstructure-coordinator)
6. [CAN Bus & Hardware Map](#-can-bus--hardware-map)
7. [Controller Bindings](#-controller-bindings)
8. [Telemetry & Logging](#-telemetry--logging)
9. [Simulation vs Real Hardware](#-simulation-vs-real-hardware)
10. [Building, Testing & Deploying](#-building-testing--deploying)

---

## 🚀 Architecture Overview

This project implements a **Decoupled I/O Architecture** separating high-level subsystem coordination from low-level motor controller APIs and sensor hardware. This enables full WPILib physics simulation, deterministic unit testing (58 passing tests), and simple hardware configuration switching.

```
                               ┌────────────────────────────────────────────────────────┐
                               │                    RobotContainer                      │
                               └───────┬─────────┬─────────┬─────────┬─────────┬────────┘
                                       │         │         │         │         │
                                       ▼         ▼         ▼         ▼         ▼
                                  SwerveDrive   Arm     Roller   Sequencer  Shooter
                                       │         │         │         │   (Flywheel+Hood+Kicker)
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
│      Real Hardware Layer              │                      │           WPILib Simulation            │
│  • SwerveModuleIOKraken (TalonFX)     │                      │  • SwerveModuleIOSim (FlywheelSim)     │
│  • GyroIO (Stub / Pigeon 2)           │                      │  • GyroIOSim (Kinematics Integration)  │
│  • ArmIOSparkMax (NEO Vortex)         │                      │  • ArmIOSim (SingleJointedArmSim)      │
│  • RollerIOKraken (2x TalonFX)        │                      │  • RollerIOSim (FlywheelSim)           │
│  • SequencerIOSparkMax (2x NEO Vortex)│                      │  • SequencerIOSim (FlywheelSim)        │
│  • ShooterIOHardware (5x TalonFX +    │                      │  • ShooterIOSim (Flywheel + Hood +     │
│    1x SPARK MAX NEO Vortex)           │                      │    Supporting Shooter Sim)             │
└───────────────────────────────────────┘                      └────────────────────────────────────────┘
```

---

## 📁 Project File Structure

```
src/
├── main/
│   ├── deploy/
│   │   └── pathplanner/           # PathPlanner paths and autonomous routines
│   │       ├── autos/             # Mobility, 3PieceHubScore, 4PieceCenterline
│   │       └── paths/             # Trajectory waypoint definitions
│   └── java/frc/robot/
│       ├── BuildConstants.java    # Git SHA, branch, and build timestamp metadata
│       ├── Constants.java         # Centralized CAN IDs, physical dimensions, gains & limits
│       ├── Main.java              # Standard WPILib robot entrypoint
│       ├── Robot.java             # AdvantageKit LoggedRobot lifecycle manager
│       ├── RobotContainer.java    # Subsystem wiring, auto chooser, teleop triggers
│       ├── subsystems/
│       │   ├── arm/               # Arm pivot: Arm.java, ArmIO.java, ArmIOSparkMax.java, ArmIOSim.java
│       │   ├── roller/            # Ground roller: Roller.java, RollerIO.java, RollerIOKraken.java, RollerIOSim.java
│       │   ├── sequencer/         # Ball feeder: Sequencer.java, SequencerIO.java, SequencerIOSparkMax.java, SequencerIOSim.java
│       │   ├── shooter/           # Shooter assembly: Shooter.java, ShooterIO.java, ShooterIOHardware.java, ShooterIOSim.java
│       │   └── swerve/            # SDS MK4i Swerve: SwerveDrive.java, SwerveModuleIO.java, SwerveModuleIOKraken.java, GyroIO.java
│       ├── superstructure/        # Mechanism coordinator: Superstructure.java, SuperstructureState.java
│       └── util/                  # AutoAim.java (ballistics interpolation & heading calculation)
└── test/java/frc/robot/
    ├── AdvantageKitLoggingTest.java # Telemetry logging & LoggedRobot tests
    ├── PathPlannerAutoTest.java     # AutoBuilder configuration, paths & named commands tests
    ├── PathPlannerStressTest.java   # Trajectory simulation, cycle stress & alliance flipping tests
    └── SubsystemsTest.java          # Mechanism kinematics, simulation dynamics & auto shoot lifecycle tests
```

---

## 🎯 Autonomous Mode

The robot's autonomous command handles scoring automatically at match start:

### 1. Default Autonomous Routine: Manual Shoot (80 RPS, 15°, 6s)
When autonomous mode is enabled (`autonomousInit`), the robot executes `m_superstructure.manual_shoot_auto()`:
1. **Flywheel**: Commands 4x Kraken X60 flywheel motors to closed-loop **80 RPS** (~4800 RPM).
2. **Hood**: Positions the adjustable hood Kraken to **15°** (`Math.toRadians(15.0)`).
3. **Kicker**: Activates the NEO Vortex supporting shooter motor.
4. **Feeder**: Runs the dual NEO Vortex sequencer feeder at **10 RPS** to deliver staged balls into the shooter.
5. **Timed Stop & Stow**: Runs continuously for **6.0 seconds**, after which the command ends, triggers `finallyDo` to stop all shooter and sequencer motors, and transitions the superstructure back to `STOW`. The default command maintains all mechanisms in a safe, stopped state for the remainder of auto.

### 2. Auto Chooser & PathPlanner Routines
- `m_autoChooser` is published to SmartDashboard / Shuffleboard under **"Auto Chooser"**.
- **Default Selection**: `"Manual Shoot (80 RPS, 15 deg, 6s)"`.
- **PathPlanner Options**: Trajectory routines (`Mobility`, `3PieceHubScore`, `4PieceCenterline`) remain selectable from the dashboard.
- **NamedCommands Registered**:
  - `"ManualShoot"`: 6-second 80 RPS / 15° manual shoot sequence.
  - `"Intake"`, `"AutoAimShoot"`, `"Stow"`, `"SpinUp"`, `"Eject"`, `"Shoot"`.

---

## ⚙️ Ball Pipeline & Control Modes

```
      1. INTAKE                       2. SEQUENCE                         3. SHOOT
 ┌──────────────────┐            ┌──────────────────┐               ┌──────────────────┐
 │   Arm + Roller   │ ─────────> │    Sequencer     │ ────────────> │  Flywheel + Hood │
 │ (Manual / Auto)  │            │ (Dual Feeder)    │               │ (+ Kicker Wheel) │
 └──────────────────┘            └──────────────────┘               └──────────────────┘
```

### 1. Active Teleop Manual Controls
For direct, predictable mechanism bringup and competition testing:
- **Manual Intake (Left Trigger)**: Deploys the arm to $+15^\circ$ and drives the intake roller at open-loop **$-8\text{ V}$** to pull balls in. On release, the roller stops and the arm remains held at $+15^\circ$.
- **Manual Shoot (Right Trigger)**: Prepares the shot at **60 RPS**, sets the hood angle to **10°**, activates the supporting kicker wheel, and runs the sequencer feeder at **10 RPS**. On release, the shooter and sequencer immediately stop and return to `STOW`.

### 2. Sequential Intake & Dynamic Auto-Aim (Automated Pipeline)
The superstructure also includes full closed-loop automation (preserved in code for vision/gyro integration):
- **Sequential Ground Intake (`intakeSequenceCommand`)**: Pivots arm to $-75^\circ$ ground intake position $\rightarrow$ verifies `m_arm.atGoal()` $\rightarrow$ spins roller at $+8\text{ V}$ $\rightarrow$ automatically stows to $0^\circ$ and stops on trigger release.
- **Dynamic Auto-Aim & Shoot (`autoAimAndShootCommand`)**: Calculates real-time distance and angle to active alliance goal from robot odometry $\rightarrow$ aligns swerve heading $\rightarrow$ interpolates flywheel velocity and hood angle from lookup tables $\rightarrow$ automatically feeds balls when heading and shooter velocity are locked.

---

## 🤖 Subsystems Breakdown

### 1. Swerve Drive
- **Geometry**: SDS MK4i modules ($21.9\text{ in} \times 21.9\text{ in}$ track width & wheelbase).
- **Drive Motors**: 4x Kraken X60 (TalonFX) with FOC voltage control ($120\text{ A}$ stator / $60\text{ A}$ supply limits).
- **Steer Motors**: 4x Kraken X44 (TalonFX) with closed-loop onboard position control and CANcoder feedback.
- **Drive Mode**: Field-relative driving when gyro is enabled; currently configured for robot-relative driving for testing.

### 2. Arm Pivot
- **Hardware**: 1x NEO Vortex brushless motor on REV SPARK MAX (`kMotorId = 30`).
- **Control**: WPILib `ProfiledPIDController` with trapezoidal motion constraints ($kP = 25.0$).
- **Presets**: $0^\circ$ STOW, $+15^\circ$ Manual Intake, $-75^\circ$ Ground Intake, $+60^\circ$ Upper Score.

### 3. Intake Roller
- **Hardware**: Dual Kraken X60 (TalonFX) (`kLeaderMotorId = 31`, `kFollowerMotorId = 32`).
- **Control**: Open-loop voltage commands ($+8\text{ V}$ intake, $-8\text{ V}$ outtake/manual intake, $+2\text{ V}$ hold).

### 4. Sequencer Feeder
- **Hardware**: Dual NEO Vortex brushless motors on REV SPARK MAX (`kLeaderMotorId = 33`, `kFollowerMotorId = 34`).
- **Control**: Velocity feedforward control ($kV = 0.12\text{ V/RPS}$, $kS = 0.25\text{ V}$, $kP = 0.1$). Configurable feeding speed (10 RPS manual/auto, 50 RPS high-speed scoring).

### 5. Shooter (Flywheel + Hood + Supporting Shooter)
6 motors total across the assembly:
- **Flywheel Array (4x Kraken X60 on TalonFX)**: IDs 35 (Leader), 36, 38, 39 (Followers). Closed-loop `VelocityVoltage(EnableFOC = true)` with Slot 0 PID and feedforward ($kP = 0.15, kV = 0.12, kS = 0.25, kA = 0.01$).
- **Adjustable Hood (1x Kraken X60 on TalonFX)**: ID 37. $80:1$ gear reduction, continuous position feedback ($0^\circ$ to $70^\circ$, $kP = 6.0$).
- **Supporting Shooter / Kicker (1x NEO Vortex on SPARK MAX)**: ID 42. Closed-loop velocity feedforward ($kV = 0.12\text{ V/RPS}, kS = 0.25\text{ V}$).

### 6. Superstructure Coordinator
Coordinates arm position, roller action, sequencer feed, and shooter speeds:
- `STOW`: Arm $0^\circ$, Roller STOP, Shooter STOP, Sequencer STOP.
- `INTAKE_GROUND`: Arm $-75^\circ$, Roller INTAKE, Shooter IDLE (20 RPS), Sequencer STOP.
- `SPIN_UP_SHOOT`: Arm $+60^\circ$, Roller HOLD, Shooter SPIN UP, Sequencer STOP.
- `SHOOT`: Arm $+60^\circ$, Roller INTAKE, Shooter SHOOT, Sequencer FEED.

---

## 📡 CAN Bus & Hardware Map

| CAN ID | Device Type | Controller / Model | Subsystem / Function | Bus |
|---|---|---|---|---|
| **1** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Left Drive | CANivore / RIO |
| **2** | Motor Controller | TalonFX (Kraken X44) | Swerve Front Left Steer | CANivore / RIO |
| **3** | Absolute Encoder | CANcoder | Swerve Front Left Steer Angle | CANivore / RIO |
| **4** | Motor Controller | TalonFX (Kraken X60) | Swerve Front Right Drive | CANivore / RIO |
| **5** | Motor Controller | TalonFX (Kraken X44) | Swerve Front Right Steer | CANivore / RIO |
| **6** | Absolute Encoder | CANcoder | Swerve Front Right Steer Angle | CANivore / RIO |
| **7** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Left Drive | CANivore / RIO |
| **8** | Motor Controller | TalonFX (Kraken X44) | Swerve Back Left Steer | CANivore / RIO |
| **9** | Absolute Encoder | CANcoder | Swerve Back Left Steer Angle | CANivore / RIO |
| **10** | Motor Controller | TalonFX (Kraken X60) | Swerve Back Right Drive | CANivore / RIO |
| **11** | Motor Controller | TalonFX (Kraken X44) | Swerve Back Right Steer | CANivore / RIO |
| **12** | Absolute Encoder | CANcoder | Swerve Back Right Steer Angle | CANivore / RIO |
| **20** | IMU | Pigeon 2 | Chassis Heading | CANivore / RIO |
| **30** | Motor Controller | REV SPARK MAX (NEO Vortex) | Arm Pivot Motor | RIO CAN |
| **31** | Motor Controller | TalonFX (Kraken X60) | Intake Roller Leader | CANivore / RIO |
| **32** | Motor Controller | TalonFX (Kraken X60) | Intake Roller Follower | CANivore / RIO |
| **33** | Motor Controller | REV SPARK MAX (NEO Vortex) | Sequencer Feeder Leader | RIO CAN |
| **34** | Motor Controller | REV SPARK MAX (NEO Vortex) | Sequencer Feeder Follower | RIO CAN |
| **35** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Leader | CANivore / RIO |
| **36** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Follower 1 | CANivore / RIO |
| **37** | Motor Controller | TalonFX (Kraken X60) | Shooter Adjustable Hood | CANivore / RIO |
| **38** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Follower 2 | CANivore / RIO |
| **39** | Motor Controller | TalonFX (Kraken X60) | Shooter Flywheel Follower 3 | CANivore / RIO |
| **42** | Motor Controller | REV SPARK MAX (NEO Vortex) | Supporting Shooter (Kicker) | RIO CAN |

---

## 🎮 Controller Bindings

### Driver Controller (Port 0 - Xbox Controller)
| Input | Action | Behavior |
|---|---|---|
| **Left Stick Y** | Translation Forward / Back | Inverted with 0.1 deadband |
| **Left Stick X** | Translation Left / Right | Inverted with 0.1 deadband |
| **Right Stick X** | Chassis Rotation | 0.1 deadband |
| **Start Button** | Reset Heading | Zeroes swerve drive heading |
| **Left Trigger (Hold)** | **Manual Intake** | Sets arm goal to $+15^\circ$ and runs roller at $-8\text{ V}$; stops roller on release |
| **Right Trigger (Hold)** | **Manual Shoot** | Spools flywheel to $60\text{ RPS}$, hood to $10^\circ$, and feeds sequencer at $10\text{ RPS}$; stops on release |
| *(Default / Idle)* | Superstructure STOW | Returns arm, roller, sequencer, and shooter to idle STOW state |

---

## 📊 Telemetry & Logging

- **AdvantageKit Logging**: Recorded to `.wpilog` on flash media and published over NetworkTables 4 (`NT4Publisher`).
- **Telemetry Topics Published**:
  - `Shooter/Flywheel Velocity (RPS)`, `Shooter/Flywheel Target (RPS)`
  - `Shooter/Hood Angle (deg)`, `Shooter/Hood Target (deg)`
  - `Shooter/Supporting Velocity (RPS)`, `Shooter/Supporting Target (RPS)`
  - `Sequencer/Velocity (RPS)`, `Sequencer/Target Velocity (RPS)`
  - `Roller/Velocity (RPS)`, `Roller/Applied Output (V)`
  - `Arm/Angle (deg)`, `Arm/Goal (deg)`
  - `AutoAim/Target Distance (m)`, `AutoAim/Target Heading (deg)`, `AutoAim/Ready To Fire`
  - `Auto Chooser`: Active autonomous routine selection

---

## 🔄 Simulation vs Real Hardware

Hardware mode switches automatically in [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java):

```java
if (RobotBase.isReal()) {
  m_sequencer = new Sequencer(new SequencerIOSparkMax(...));
  m_arm = new Arm(new ArmIOSparkMax(...));
  m_roller = new Roller(new RollerIOKraken(...));
  m_shooter = new Shooter(new ShooterIOHardware(...));
  m_swerveDrive = new SwerveDrive(new GyroIO() {}, new SwerveModuleIOKraken(...), ...);
} else {
  m_sequencer = new Sequencer(new SequencerIOSim());
  m_arm = new Arm(new ArmIOSim());
  m_roller = new Roller(new RollerIOSim());
  m_shooter = new Shooter(new ShooterIOSim());
  m_swerveDrive = new SwerveDrive(new GyroIOSim(), new SwerveModuleIOSim(), ...);
}
```

---

## 🛠️ Building, Testing & Deploying

### Build Project
```bash
./gradlew build
```

### Run Unit Tests (58 tests)
```bash
./gradlew test
```

### Run WPILib Desktop Simulation
```bash
./gradlew simulateJava
```

### Deploy to Robot
```bash
./gradlew deploy
```
