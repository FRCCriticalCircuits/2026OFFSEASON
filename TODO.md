# 📝 Robot Tuning & Setup Checklist (TODO.md)

This document tracks all constants, physical dimensions, CAN IDs, current limits, and PID/feedforward gains in [`Constants.java`](src/main/java/frc/robot/Constants.java).

---

## 📋 Table of Contents
1. [CAN Bus & Device ID Mapping](#1-can-bus--device-id-mapping)
2. [Swerve Drivetrain Setup & Calibration](#2-swerve-drivetrain-setup--calibration)
3. [Arm Mechanism Tuning](#3-arm-mechanism-tuning)
4. [Sequencer Mechanism Tuning](#4-sequencer-mechanism-tuning)
5. [Roller (Intake) Tuning](#5-roller-intake-tuning)
6. [Shooter (Flywheel + Hood) Tuning](#6-shooter-flywheel--hood-tuning)
7. [Auto-Aim & Field Calibration](#7-auto-aim--field-calibration)
8. [Superstructure State Machine Setpoints](#8-superstructure-state-machine-setpoints)

---

## 1. CAN Bus & Device ID Mapping

Device IDs configured from `TunerConstants.java` and `Constants.java`:

- [x] **CAN Bus Name**: `kCANBusName = ""` (Default RIO CAN bus)
- [x] **Swerve Pigeon 2 IMU**: `kPigeon2CanId = 20`
- [x] **Front Left Module**:
  - [x] Drive Motor: `kFrontLeftDriveMotorId = 1` (Kraken X60)
  - [x] Steer Motor: `kFrontLeftSteerMotorId = 2` (Kraken X44)
  - [x] CANcoder: `kFrontLeftCANcoderId = 3`
- [x] **Front Right Module**:
  - [x] Drive Motor: `kFrontRightDriveMotorId = 4` (Kraken X60)
  - [x] Steer Motor: `kFrontRightSteerMotorId = 5` (Kraken X44)
  - [x] CANcoder: `kFrontRightCANcoderId = 6`
- [x] **Back Left Module**:
  - [x] Drive Motor: `kBackLeftDriveMotorId = 7` (Kraken X60)
  - [x] Steer Motor: `kBackLeftSteerMotorId = 8` (Kraken X44)
  - [x] CANcoder: `kBackLeftCANcoderId = 9`
- [x] **Back Right Module**:
  - [x] Drive Motor: `kBackRightDriveMotorId = 10` (Kraken X60)
  - [x] Steer Motor: `kBackRightSteerMotorId = 11` (Kraken X44)
  - [x] CANcoder: `kBackRightCANcoderId = 12`
- [x] **Arm (Intake Deploy) Motor**: `ArmConstants.kMotorId = 30` (NEO Vortex on SPARK MAX)
- [x] **Roller (Intake Leader) Motor**: `RollerConstants.kLeaderMotorId = 31` (Kraken X60)
- [x] **Roller (Intake Follower) Motor**: `RollerConstants.kFollowerMotorId = 32` (Kraken X60)
- [x] **Sequencer Leader Motor**: `SequencerConstants.kLeaderMotorId = 33` (NEO Vortex on SPARK MAX)
- [x] **Sequencer Follower Motor**: `SequencerConstants.kFollowerMotorId = 34` (NEO Vortex on SPARK MAX)
- [x] **Shooter Flywheel Leader Motor**: `ShooterConstants.kFlywheelLeaderMotorId = 35` (Kraken X60)
- [x] **Shooter Flywheel Follower 1 Motor**: `ShooterConstants.kFlywheelFollower1MotorId = 36` (Kraken X60)
- [x] **Shooter Flywheel Follower 2 Motor**: `ShooterConstants.kFlywheelFollower2MotorId = 38` (Kraken X60)
- [x] **Shooter Flywheel Follower 3 Motor**: `ShooterConstants.kFlywheelFollower3MotorId = 39` (Kraken X60)
- [x] **Shooter Flywheel Follower 4 Motor**: `ShooterConstants.kFlywheelFollower4MotorId = 37` (Kraken X60)
- [x] **Shooter Hood Motor**: `ShooterConstants.kHoodMotorId = 42` (NEO Vortex on SPARK MAX)

---

## 2. Swerve Drivetrain Setup & Calibration

### Physical Dimensions
| Variable | Configured Value | Unit | Source |
|---|---|---|---|
| `kTrackWidthMeters` | `0.55626` ($21.9\text{ in}$) | meters | TunerConstants ($\pm 10.95\text{ in}$) |
| `kWheelbaseMeters` | `0.55626` ($21.9\text{ in}$) | meters | TunerConstants ($\pm 10.95\text{ in}$) |
| `kWheelRadiusMeters` | `0.0508` ($2.0\text{ in}$) | meters | TunerConstants (`kWheelRadius = Inches.of(2)`) |

### CANcoder Magnet Offsets & Inversions
| Variable | Configured Value | Unit | Source |
|---|---|---|---|
| `kFrontLeftCANcoderOffsetRotations` | `-0.252197265625` | rotations | TunerConstants |
| `kFrontRightCANcoderOffsetRotations` | `-0.48046875` | rotations | TunerConstants |
| `kBackLeftCANcoderOffsetRotations` | `0.327880859375` | rotations | TunerConstants |
| `kBackRightCANcoderOffsetRotations` | `-0.247314453125` | rotations | TunerConstants |
| `kFrontLeftDriveInverted` | `false` | boolean | TunerConstants |
| `kFrontRightDriveInverted` | `true` | boolean | TunerConstants |
| `kBackLeftDriveInverted` | `false` | boolean | TunerConstants |
| `kBackRightDriveInverted` | `true` | boolean | TunerConstants |

### Drivetrain Gains & Current Limits
| Variable | Configured Value | Description |
|---|---|---|
| `kMaxSpeedMetersPerSecond` | `5.12` | TunerConstants (`kSpeedAt12Volts`) |
| `kDriveGearRatio` | `6.026785714285714` | TunerConstants |
| `kSteerGearRatio` | `26.09090909090909` | TunerConstants |
| `kDriveStatorCurrentLimitAmps` | `120.0` | Peak slip torque limit |
| `kDriveSupplyCurrentLimitAmps` | `60.0` | Supply current limit |
| `kSteerStatorCurrentLimitAmps` | `40.0` | Kraken X44 steer stator limit |
| `kSteerSupplyCurrentLimitAmps` | `30.0` | Kraken X44 steer supply limit |
| `kSteerProportionalGain` ($kP$) | `59.5` | TunerConstants |
| `kSteerDerivativeGain` ($kD$) | `0.075` | TunerConstants |
| `kDriveProportionalGain` ($kP$) | `0.05` | TunerConstants |

---

## 3. Arm Mechanism Tuning

| Variable | Configured Value | Description |
|---|---|---|
| `kMotorId` | `30` | Kraken X60 intake pivot |
| `kProportionalGain` ($kP$) | `25.0` | Position proportional gain |
| `kSupplyCurrentLimitAmps` | `40.0` | Supply current limit |
| `kMinAngleRadians` | $-90^\circ$ | Soft limit |
| `kMaxAngleRadians` | $+90^\circ$ | Soft limit |

---

## 4. Sequencer Mechanism Tuning (Spinning Feeder)

| Variable | Configured Value | Description |
|---|---|---|
| `kMotorId` | `34` | Feeder motor ID |
| `kSupplyCurrentLimitAmps` | `50.0` | Supply current limit |
| `kVelocityGain` ($kV$) | `0.12` | Feedforward gain (volts per RPS) |
| `kStaticGain` ($kS$) | `0.25` | Friction compensation |
| `kFeedVelocityRotationsPerSecond` | `50.0` | Target feeding speed ($\approx 3000\text{ RPM}$) |

---

## 5. Roller (Intake) Tuning

| Variable | Configured Value | Description |
|---|---|---|
| `kLeaderMotorId` | `31` | Intake leader Kraken X60 motor ID |
| `kFollowerMotorId` | `32` | Intake follower Kraken X60 motor ID (Aligned) |
| `kStatorCurrentLimitAmps` | `60.0` | Peak torque stator current limit |
| `kSupplyCurrentLimitAmps` | `60.0` | Supply current limit |
| `kIntakeAppliedVolts` | `8.0` | Forward intake voltage |
| `kEjectAppliedVolts` | `-8.0` | Reverse eject voltage |
| `kHoldAppliedVolts` | `2.0` | Retention voltage |

---

## 6. Shooter (Flywheel + Hood + Supporting Shooter) Tuning

| Variable | Configured Value | Description |
|---|---|---|
| `kFlywheelLeaderMotorId` | `35` | Flywheel leader Kraken X60 motor |
| `kFlywheelFollower1MotorId` | `36` | Flywheel follower 1 Kraken X60 motor (Opposed) |
| `kFlywheelFollower2MotorId` | `38` | Flywheel follower 2 Kraken X60 motor (Aligned) |
| `kFlywheelFollower3MotorId` | `39` | Flywheel follower 3 Kraken X60 motor (Opposed) |
| `kFlywheelProportionalGain` ($kP$) | `0.15` | Flywheel velocity $kP$ |
| `kFlywheelSupplyCurrentLimitAmps` | `60.0` | Flywheel supply limit |
| `kHoodMotorId` | `37` | Adjustable Hood Kraken X60 motor |
| `kHoodProportionalGain` ($kP$) | `2.5` | Hood position $kP$ |
| `kHoodSupplyCurrentLimitAmps` | `40.0` | Hood supply limit |
| `kHoodMinAngleRadians` | $0^\circ$ | Hood minimum angle |
| `kHoodMaxAngleRadians` | $60^\circ$ | Hood maximum angle |
| `kSupportingShooterMotorId` | `42` | Supporting shooter kicker NEO Vortex (SPARK MAX) |
| `kSupportingShooterSmartCurrentLimitAmps` | `60` | Supporting shooter smart limit |
| `kSupportingShooterTargetVelocityRotationsPerSecond` | `60.0` | Supporting shooter speed ($\approx 3600\text{ RPM}$) |

---

## 7. Auto-Aim & Field Calibration

| Variable | Configured Value | Description |
|---|---|---|
| `kBlueGoalLocation` | `(4.626m, 4.035m)` | Blue scoring hub center ($182.11\text{ in}$) |
| `kRedGoalLocation` | `(11.915m, 4.035m)` | Red scoring hub center ($651.22 - 182.11\text{ in}$) |
| `kHeadingProportionalGain` | `9.0` | Heading tracking $kP$ |
| `kHeadingDerivativeGain` | `0.15` | Heading damping $kD$ |
| `kHeadingToleranceRadians` | $2.0^\circ$ | Heading alignment tolerance |
| `kTargetAimOffset` | $180^\circ$ (`Rotation2d.k180deg`) | Back-facing shooter offset |
| `kAutoAimMaxSpeedMultiplier` | `0.70` | 30% reduction during auto-aim |

---

## 8. Superstructure State Machine Setpoints

| Preset Name | Arm Angle | Sequencer | Roller | Shooter |
|---|---|---|---|---|
| **`STOW`** | $0^\circ$ | `STOP` | `STOP` | `STOP` |
| **`INTAKE_GROUND`** | $-75^\circ$ | `STOP` | `INTAKE` ($8\text{V}$) | `IDLE` |
| **`SPIN_UP_SHOOT`** | $+60^\circ$ | `STOP` | `HOLD` ($2\text{V}$) | Spooling / Positioning |
| **`SHOOT`** | $+60^\circ$ | **`FEED`** ($50\text{ RPS}$) | `INTAKE` ($8\text{V}$) | At Speed / Angle |
