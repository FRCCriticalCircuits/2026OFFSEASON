# 📝 Robot Tuning & Setup Checklist (TODO.md)

This document tracks all constants, physical dimensions, CAN IDs, current limits, and PID/feedforward gains in [`Constants.java`](src/main/java/frc/robot/Constants.java) that must be calibrated on the physical robot.

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

Configure device IDs to match the physical CANivore bus topology.

- [ ] **CAN Bus Name**: `kCANBusName = "canivore"` (Default: `"canivore"`)
- [ ] **Swerve Pigeon 2 IMU**: `kPigeon2CanId` (Default: `0`)
- [ ] **Front Left Module**:
  - [ ] Drive Motor: `kFrontLeftDriveMotorId` (Default: `1`)
  - [ ] Steer Motor: `kFrontLeftSteerMotorId` (Default: `2`)
  - [ ] CANcoder: `kFrontLeftCANcoderId` (Default: `3`)
- [ ] **Front Right Module**:
  - [ ] Drive Motor: `kFrontRightDriveMotorId` (Default: `4`)
  - [ ] Steer Motor: `kFrontRightSteerMotorId` (Default: `5`)
  - [ ] CANcoder: `kFrontRightCANcoderId` (Default: `6`)
- [ ] **Back Left Module**:
  - [ ] Drive Motor: `kBackLeftDriveMotorId` (Default: `7`)
  - [ ] Steer Motor: `kBackLeftSteerMotorId` (Default: `8`)
  - [ ] CANcoder: `kBackLeftCANcoderId` (Default: `9`)
- [ ] **Back Right Module**:
  - [ ] Drive Motor: `kBackRightDriveMotorId` (Default: `13`)
  - [ ] Steer Motor: `kBackRightSteerMotorId` (Default: `11`)
  - [ ] CANcoder: `kBackRightCANcoderId` (Default: `12`)
- [ ] **Arm Pivot Motor**: `ArmConstants.kMotorId` (Default: `10`)
- [ ] **Sequencer Lift Motor**: `SequencerConstants.kMotorId` (Default: `20`)
- [ ] **Roller Motor**: `RollerConstants.kMotorId` (Default: `30`)
- [ ] **Shooter Flywheel Motors**:
  - [ ] Leader Motor: `ShooterConstants.kFlywheelLeaderMotorId` (Default: `40`)
  - [ ] Follower Motor: `ShooterConstants.kFlywheelFollowerMotorId` (Default: `41`)
- [ ] **Shooter Hood Motor**:
  - [ ] Hood Motor: `ShooterConstants.kHoodMotorId` (Default: `42`)

---

## 2. Swerve Drivetrain Setup & Calibration

### Physical Dimensions
| Variable | Default Value | Unit | Description / Tuning Procedure |
|---|---|---|---|
| `kTrackWidthMeters` | `0.0` | meters | Center-to-center distance between left and right wheels. |
| `kWheelbaseMeters` | `0.0` | meters | Center-to-center distance between front and back wheels. |
| `kWheelRadiusMeters` | `0.0` | meters | Measured wheel radius ($2\text{ in} = 0.0508\text{ m}$). |

- [ ] Measure and set `kTrackWidthMeters`.
- [ ] Measure and set `kWheelbaseMeters`.
- [ ] Measure and set `kWheelRadiusMeters`.

### CANcoder Magnet Offsets & Inversions
Point all module bevel gears in the same direction (e.g. facing left or right), read the absolute rotation value in Phoenix Tuner X, and record the opposite offset.

| Variable | Default Value | Unit | Description |
|---|---|---|---|
| `kFrontLeftCANcoderOffsetRotations` | `0.0` | rotations | Zero offset for FL steering encoder. |
| `kFrontRightCANcoderOffsetRotations` | `0.0` | rotations | Zero offset for FR steering encoder. |
| `kBackLeftCANcoderOffsetRotations` | `0.0` | rotations | Zero offset for BL steering encoder. |
| `kBackRightCANcoderOffsetRotations` | `0.0` | rotations | Zero offset for BR steering encoder. |
| `kFrontLeftDriveInverted` | `false` | boolean | Set `true` if FL wheel drives backward when commanded forward. |
| `kFrontRightDriveInverted` | `false` | boolean | Set `true` if FR wheel drives backward when commanded forward. |
| `kBackLeftDriveInverted` | `false` | boolean | Set `true` if BL wheel drives backward when commanded forward. |
| `kBackRightDriveInverted` | `false` | boolean | Set `true` if BR wheel drives backward when commanded forward. |

- [ ] Zero Front Left CANcoder offset.
- [ ] Zero Front Right CANcoder offset.
- [ ] Zero Back Left CANcoder offset.
- [ ] Zero Back Right CANcoder offset.
- [ ] Verify drive motor directions.

### Drivetrain Gains & Current Limits
| Variable | Default Value | Description |
|---|---|---|
| `kMaxSpeedMetersPerSecond` | `4.5` | Max robot translation velocity. |
| `kMaxAngularSpeedRadiansPerSecond` | $2\pi$ ($\approx 6.28$) | Max robot angular rotation speed. |
| `kDriveStatorCurrentLimitAmps` | `80.0` | Drive motor peak torque limit. |
| `kDriveSupplyCurrentLimitAmps` | `40.0` | Drive motor battery protection limit. |
| `kSteerStatorCurrentLimitAmps` | `40.0` | Steer motor peak torque limit. |
| `kSteerSupplyCurrentLimitAmps` | `20.0` | Steer motor battery protection limit. |
| `kDriveProportionalGain` | `0.1` | Drive velocity feedback proportional gain ($kP$). |
| `kDriveVelocityGain` | `0.12` | Drive velocity feedforward gain ($kV$). |
| `kSteerProportionalGain` | `100.0` | Steering position proportional gain ($kP$). |
| `kSteerDerivativeGain` | `0.5` | Steering position derivative gain ($kD$). |

- [ ] Tune steer $kP$ and $kD$ to prevent oscillation while maintaining snappy heading response.
- [ ] Tune drive $kV$ and $kP$ for accurate velocity tracking.

---

## 3. Arm Mechanism Tuning

### Physical Properties & Limits
| Variable | Default Value | Unit | Description |
|---|---|---|---|
| `kGearRatio` | `50.0` | ratio | Total mechanical gear reduction from Kraken motor to arm axle. |
| `kArmLengthMeters` | `0.5` | meters | Center of rotation to end of arm. |
| `kArmMassKilograms` | `3.0` | kg | Estimated / CAD mass of the moving arm assembly. |
| `kMinAngleRadians` | $-90^\circ$ | radians | Minimum allowable software travel limit. |
| `kMaxAngleRadians` | $+90^\circ$ | radians | Maximum allowable software travel limit. |

- [ ] Set exact mechanical gear ratio.
- [ ] Measure physical travel limits and set software soft limits.

### Control Gains (Profiled PID + Feedforward)
| Variable | Default Value | Description / Tuning Procedure |
|---|---|---|
| `kGravityGain` ($kG$) | `0.0` | Voltage required to hold arm horizontally against gravity ($0^\circ$). |
| `kStaticGain` ($kS$) | `0.0` | Voltage required to overcome static friction. |
| `kVelocityGain` ($kV$) | `0.0` | Voltage per unit of target angular velocity. |
| `kAccelerationGain` ($kA$) | `0.0` | Voltage per unit of target angular acceleration. |
| `kProportionalGain` ($kP$) | `0.0` | Feedback proportional gain. |
| `kIntegralGain` ($kI$) | `0.0` | Feedback integral gain. |
| `kDerivativeGain` ($kD$) | `0.0` | Feedback derivative gain (dampens overshoot). |
| `kMaxVelocityRadiansPerSecond` | $\pi$ ($180^\circ/\text{s}$) | Trapezoid profile max velocity limit. |
| `kMaxAccelerationRadiansPerSecondSquared` | $2\pi$ ($360^\circ/\text{s}^2$) | Trapezoid profile max acceleration limit. |
| `kToleranceRadians` | $2.0^\circ$ | Position tolerance threshold to declare `atGoal()`. |

- [ ] Measure $kG$: Apply manual voltage until the arm balances horizontally without drifting.
- [ ] Tune $kP$ and $kD$ using SysId or manual test steps.
- [ ] Adjust trapezoid profile velocity and acceleration for smooth movement.

---

## 4. Sequencer Mechanism Tuning

### Physical Properties & Limits
| Variable | Default Value | Unit | Description |
|---|---|---|---|
| `kGearRatio` | `10.0` | ratio | Total gear reduction between motor and lift drum/spool. |
| `kDrumRadiusMeters` | `0.025` ($2.5\text{ cm}$) | meters | Radius of the drum or pitch radius of the pulley/sprocket. |
| `kCarriageMassKilograms` | `5.0` | kg | Mass of the moving stage. |
| `kMinHeightMeters` | `0.0` | meters | Bottom hard-stop height. |
| `kMaxHeightMeters` | `1.3` | meters | Top maximum extension height. |

- [ ] Set exact drum radius and gear ratio for accurate linear meter conversion.
- [ ] Confirm bottom limit switch resets encoder to $0.0\text{ m}$.

### Control Gains & Feed Voltages
| Variable | Default Value | Description |
|---|---|---|
| `kGravityGain` ($kG$) | `0.0` | Voltage to counteract gravity on carriage. |
| `kProportionalGain` ($kP$) | `0.0` | Height position feedback gain. |
| `kDerivativeGain` ($kD$) | `0.0` | Height position damping gain. |
| `kMaxVelocityMetersPerSecond` | `1.0` | Max carriage lift speed. |
| `kMaxAccelerationMetersPerSecondSquared` | `2.0` | Max carriage acceleration. |
| `kFeedToShooterAppliedVolts` | `10.0` | Voltage applied when feeding balls into the spinning flywheel. |
| `kIndexBallsAppliedVolts` | `6.0` | Voltage applied when indexing balls from the intake. |
| `kReverseFeedAppliedVolts` | `-6.0` | Voltage applied when clearing jams / outtaking. |

- [ ] Tune $kG$ and $kP$ for accurate height holding.
- [ ] Calibrate `kFeedToShooterAppliedVolts` to ensure consistent ball transfer without jamming.

---

## 5. Roller (Intake) Tuning

| Variable | Default Value | Description |
|---|---|---|
| `kGearRatio` | `3.0` | Gear ratio between Kraken motor and intake rollers. |
| `kStatorCurrentLimitAmps` | `60.0` | Peak torque limit for intake rollers. |
| `kSupplyCurrentLimitAmps` | `40.0` | Continuous battery draw limit. |
| `kIntakeAppliedVolts` | `10.0` | Forward intake speed for pulling balls into the robot. |
| `kOuttakeAppliedVolts` | `-8.0` | Reverse speed for ejecting balls. |
| `kHoldAppliedVolts` | `2.0` | Low retention voltage for holding a captured ball in place. |

- [ ] Test intake roller speed with game pieces to ensure positive grip without wheel slip.
- [ ] Test retention hold voltage.

---

## 6. Shooter (Flywheel + Hood) Tuning

### A. Flywheel Motor Tuning
| Variable | Default Value | Unit | Description |
|---|---|---|---|
| `kFlywheelProportionalGain` ($kP$) | `0.12` | — | Velocity closed-loop proportional gain on TalonFX. |
| `kFlywheelStaticGain` ($kS$) | `0.25` | volts | Voltage to overcome flywheel friction. |
| `kFlywheelVelocityGain` ($kV$) | `0.12` | volts / RPS | Velocity feedforward gain ($12\text{ V} / \text{Max RPS}$). |
| `kFlywheelAccelerationGain` ($kA$) | `0.01` | volts / $\text{RPS}^2$ | Acceleration feedforward gain for rapid spin-up. |
| `kFlywheelTargetVelocityRotationsPerSecond` | `70.0` | RPS ($\approx 4200\text{ RPM}$) | Default scoring launch speed. |
| `kFlywheelIdleVelocityRotationsPerSecond` | `20.0` | RPS ($\approx 1200\text{ RPM}$) | Idle pre-spin speed to reduce spool latency. |
| `kFlywheelToleranceRotationsPerSecond` | `2.5` | RPS | Acceptable speed window to declare `atTargetFlywheelSpeed()`. |

- [ ] Calculate initial $kV$: $\frac{12.0\text{ V}}{\text{Free Speed RPS}} = \frac{12.0}{100.0} \approx 0.12$.
- [ ] Tune $kP$ until flywheel recovers rapidly when a ball passes through without surging or oscillating.

### B. Adjustable Hood Motor Tuning
| Variable | Default Value | Unit | Description |
|---|---|---|---|
| `kHoodGearRatio` | `50.0` | ratio | Gear reduction between Kraken motor and adjustable hood pivot. |
| `kHoodMinAngleRadians` | $0^\circ$ ($0.0\text{ rad}$) | radians | Minimum hood angle soft limit. |
| `kHoodMaxAngleRadians` | $45^\circ$ ($0.785\text{ rad}$) | radians | Maximum hood angle soft limit. |
| `kHoodStatorCurrentLimitAmps` | `40.0` | amps | Peak stator current limit for hood motor. |
| `kHoodSupplyCurrentLimitAmps` | `20.0` | amps | Supply current limit. |
| `kHoodProportionalGain` ($kP$) | `50.0` | — | Closed-loop position proportional gain on TalonFX. |
| `kHoodDerivativeGain` ($kD$) | `0.5` | — | Closed-loop position derivative damping gain. |
| `kHoodToleranceRadians` | $1.0^\circ$ | radians | Hood angular tolerance threshold. |

- [ ] Set exact mechanical gear ratio for the hood.
- [ ] Set physical hard stops and software travel limits.
- [ ] Tune position PID ($kP$, $kD$) for quick and stable hood positioning.

---

## 7. Auto-Aim & Field Calibration

| Variable | Default Value | Description |
|---|---|---|
| `kBlueGoalLocation` | `(0.0, 5.55)` m | Exact field coordinates of Blue Alliance Goal. |
| `kRedGoalLocation` | `(16.54, 5.55)` m | Exact field coordinates of Red Alliance Goal. |
| `kHeadingProportionalGain` | `5.0` | Swerve drive rotation $kP$ for heading tracking. |
| `kHeadingDerivativeGain` | `0.2` | Swerve drive rotation $kD$ for damping heading overshoot. |
| `kHeadingToleranceRadians` | $1.5^\circ$ | Heading error window to declare `headingAligned`. |
| `kAutoAimMaxSpeedMultiplier` | `0.70` | 30% reduction of max translation speed during auto-aim for safety. |
| `AutoAim.m_flywheelSpeedMap` | $(1.5\text{m}, 55\text{RPS}) \dots (6.5\text{m}, 94\text{RPS})$ | Ballistics distance-to-flywheel velocity curve. |
| `AutoAim.m_hoodAngleMap` | $(1.5\text{m}, 12^\circ) \dots (6.5\text{m}, 42^\circ)$ | Ballistics distance-to-hood angle curve. |

- [ ] Measure exact field goal $(X, Y)$ coordinates for current season field layout.
- [ ] Test shoot from $1.5\text{m}, 2.5\text{m}, 3.5\text{m}, 4.5\text{m}, 5.5\text{m}$ and calibrate the empirical map in [`AutoAim.java`](src/main/java/frc/robot/util/AutoAim.java).

---

## 8. Superstructure State Machine Setpoints

| Preset Name | Arm Angle | Sequencer Height | Notes |
|---|---|---|---|
| **`STOW`** | $0^\circ$ ($0.00\text{ rad}$) | $0.00\text{ m}$ | Fully retracted starting/travel pose. |
| **`INTAKE_GROUND`** | $-45^\circ$ ($-0.785\text{ rad}$) | $0.10\text{ m}$ | Floor intake position. |
| **`SPIN_UP_SHOOT`** | $+60^\circ$ ($+1.047\text{ rad}$) | $0.80\text{ m}$ | Staging pose while auto-aiming. |

- [ ] Measure physical arm angle for floor collection.
- [ ] Verify transitions between states avoid internal mechanism collisions.
