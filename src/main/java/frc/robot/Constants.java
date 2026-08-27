// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static final). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or its individual members) to use them.
 */
public final class Constants {

  /** CAN bus name for all CTRE devices (CANivore). */
  public static final String kCANBusName = "canivore";

  public static final class DriverConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ArmConstants {
    // CAN ID
    public static final int kMotorId = 10; // TODO: Set to actual CAN ID

    // Physical Constants
    public static final double kGearRatio = 50.0; // TODO: Tune this value
    public static final double kArmLengthMeters = 0.5; // TODO: Tune this value
    public static final double kArmMassKilograms = 3.0; // TODO: Tune this value
    public static final double kMinAngleRadians = Math.toRadians(-90.0); // TODO: Tune this value
    public static final double kMaxAngleRadians = Math.toRadians(90.0); // TODO: Tune this value

    // Kraken X60 Current Limits
    public static final double kStatorCurrentLimitAmps = 60.0; // TODO: Tune this value
    public static final double kSupplyCurrentLimitAmps = 40.0; // TODO: Tune this value

    // Control Gains (PID + Feedforward) — used by WPILib ProfiledPIDController & ArmFeedforward
    public static final double kProportionalGain = 0.0; // TODO: Tune this value
    public static final double kIntegralGain = 0.0; // TODO: Tune this value
    public static final double kDerivativeGain = 0.0; // TODO: Tune this value
    public static final double kStaticGain = 0.0; // TODO: Tune this value
    public static final double kGravityGain = 0.0; // TODO: Tune this value — gravity compensation (volts)
    public static final double kVelocityGain = 0.0; // TODO: Tune this value
    public static final double kAccelerationGain = 0.0; // TODO: Tune this value

    // Profile Constraints
    public static final double kMaxVelocityRadiansPerSecond = Math.PI; // TODO: Tune this value
    public static final double kMaxAccelerationRadiansPerSecondSquared = Math.PI * 2.0; // TODO: Tune this value
    public static final double kToleranceRadians = Math.toRadians(2.0); // TODO: Tune this value
  }

  public static final class RollerConstants {
    // CAN ID
    public static final int kMotorId = 30; // TODO: Set to actual CAN ID

    // Physical Constants
    public static final double kGearRatio = 3.0; // TODO: Tune this value

    // Kraken X60 Current Limits
    public static final double kStatorCurrentLimitAmps = 60.0; // TODO: Tune this value
    public static final double kSupplyCurrentLimitAmps = 40.0; // TODO: Tune this value

    // Applied Voltages for Roller Actions
    public static final double kIntakeAppliedVolts = 10.0; // TODO: Tune this value
    public static final double kOuttakeAppliedVolts = -8.0; // TODO: Tune this value
    public static final double kHoldAppliedVolts = 2.0; // TODO: Tune this value
  }

  public static final class SequencerConstants {
    // CAN ID
    public static final int kMotorId = 20; // TODO: Set to actual CAN ID

    // Physical Constants
    public static final double kGearRatio = 10.0; // TODO: Tune this value
    public static final double kDrumRadiusMeters = 0.025; // TODO: Tune this value
    public static final double kCarriageMassKilograms = 5.0; // TODO: Tune this value
    public static final double kMinHeightMeters = 0.0; // TODO: Tune this value
    public static final double kMaxHeightMeters = 1.3; // TODO: Tune this value

    // Kraken X60 Current Limits
    public static final double kStatorCurrentLimitAmps = 80.0; // TODO: Tune this value
    public static final double kSupplyCurrentLimitAmps = 40.0; // TODO: Tune this value

    // Control Gains (PID + Feedforward) — used by WPILib ProfiledPIDController & ElevatorFeedforward
    public static final double kProportionalGain = 0.0; // TODO: Tune this value
    public static final double kIntegralGain = 0.0; // TODO: Tune this value
    public static final double kDerivativeGain = 0.0; // TODO: Tune this value
    public static final double kStaticGain = 0.0; // TODO: Tune this value
    public static final double kGravityGain = 0.0; // TODO: Tune this value — gravity compensation (volts)
    public static final double kVelocityGain = 0.0; // TODO: Tune this value
    public static final double kAccelerationGain = 0.0; // TODO: Tune this value

    // Profile Constraints
    public static final double kMaxVelocityMetersPerSecond = 1.0; // TODO: Tune this value
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0; // TODO: Tune this value
    public static final double kToleranceMeters = 0.01; // TODO: Tune this value

    // Feed / Indexing Operating Voltages
    public static final double kFeedToShooterAppliedVolts = 10.0; // TODO: Tune this value
    public static final double kIndexBallsAppliedVolts = 6.0; // TODO: Tune this value
    public static final double kReverseFeedAppliedVolts = -6.0; // TODO: Tune this value
  }

  public static final class ShooterConstants {
    // ─── Flywheel Motors (Dual Kraken X60) ───────────────────────────────────
    public static final int kFlywheelLeaderMotorId = 40; // TODO: Set to actual CAN ID
    public static final int kFlywheelFollowerMotorId = 41; // TODO: Set to actual CAN ID
    public static final double kFlywheelGearRatio = 1.0; // TODO: Tune this value

    public static final double kFlywheelStatorCurrentLimitAmps = 80.0; // TODO: Tune this value
    public static final double kFlywheelSupplyCurrentLimitAmps = 40.0; // TODO: Tune this value

    // Flywheel Velocity Closed-Loop PID & Feedforward Gains (Slot 0 on TalonFX)
    public static final double kFlywheelProportionalGain = 0.12; // TODO: Tune this value
    public static final double kFlywheelIntegralGain = 0.0; // TODO: Tune this value
    public static final double kFlywheelDerivativeGain = 0.0; // TODO: Tune this value
    public static final double kFlywheelStaticGain = 0.25; // TODO: Tune this value
    public static final double kFlywheelVelocityGain = 0.12; // TODO: Tune this value
    public static final double kFlywheelAccelerationGain = 0.01; // TODO: Tune this value

    // Flywheel Target Velocity Setpoints (rotations per second)
    public static final double kFlywheelTargetVelocityRotationsPerSecond = 70.0; // ~4200 RPM // TODO: Tune this value
    public static final double kFlywheelIdleVelocityRotationsPerSecond = 20.0; // ~1200 RPM idle spool // TODO: Tune this value
    public static final double kFlywheelToleranceRotationsPerSecond = 2.5; // TODO: Tune this value

    // ─── Hood Motor (Kraken X60) ─────────────────────────────────────────────
    public static final int kHoodMotorId = 42; // TODO: Set to actual CAN ID
    public static final double kHoodGearRatio = 50.0; // TODO: Tune this value

    public static final double kHoodMinAngleRadians = Math.toRadians(0.0); // TODO: Tune this value
    public static final double kHoodMaxAngleRadians = Math.toRadians(45.0); // TODO: Tune this value

    public static final double kHoodStatorCurrentLimitAmps = 40.0; // TODO: Tune this value
    public static final double kHoodSupplyCurrentLimitAmps = 20.0; // TODO: Tune this value

    // Hood Position Closed-Loop PID Gains (Slot 0 on TalonFX)
    public static final double kHoodProportionalGain = 50.0; // TODO: Tune this value
    public static final double kHoodIntegralGain = 0.0; // TODO: Tune this value
    public static final double kHoodDerivativeGain = 0.5; // TODO: Tune this value
    public static final double kHoodToleranceRadians = Math.toRadians(1.0); // TODO: Tune this value

    // Hood Angle Preset Targets
    public static final double kHoodStowAngleRadians = Math.toRadians(0.0); // TODO: Tune this value
    public static final double kHoodLowGoalAngleRadians = Math.toRadians(15.0); // TODO: Tune this value
    public static final double kHoodHighGoalAngleRadians = Math.toRadians(35.0); // TODO: Tune this value
  }

  public static final class SwerveConstants {
    // ─── IMU ─────────────────────────────────────────────────────────────────
    public static final int kPigeon2CanId = 0; // TODO: Set to actual CAN ID

    // ─── Robot Dimensions ────────────────────────────────────────────────────
    public static final double kTrackWidthMeters = 0.0; // TODO: Set to actual track width (m)
    public static final double kWheelbaseMeters = 0.0; // TODO: Set to actual wheelbase (m)

    // ─── Module Gear Ratios (SDS MK4i L2) ────────────────────────────────────
    public static final double kDriveGearRatio = 6.75; // MK4i L2 drive ratio
    public static final double kSteerGearRatio = 150.0 / 7.0; // MK4i L2 steer ratio

    // ─── Wheel ───────────────────────────────────────────────────────────────
    public static final double kWheelRadiusMeters = 0.0; // TODO: Set to actual wheel radius (m)
    public static final double kWheelCircumferenceMeters = 2.0 * Math.PI * kWheelRadiusMeters;

    // ─── Speed Limits ────────────────────────────────────────────────────────
    public static final double kMaxSpeedMetersPerSecond = 4.5; // TODO: Tune this value
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2.0; // TODO: Tune this value

    // ─── Drive Motor Current Limits ──────────────────────────────────────────
    public static final double kDriveStatorCurrentLimitAmps = 80.0; // TODO: Tune this value
    public static final double kDriveSupplyCurrentLimitAmps = 40.0; // TODO: Tune this value

    // ─── Steer Motor Current Limits ──────────────────────────────────────────
    public static final double kSteerStatorCurrentLimitAmps = 40.0; // TODO: Tune this value
    public static final double kSteerSupplyCurrentLimitAmps = 20.0; // TODO: Tune this value

    // ─── Drive Motor PID Gains ───────────────────────────────────────────────
    public static final double kDriveProportionalGain = 0.1; // TODO: Tune this value
    public static final double kDriveIntegralGain = 0.0; // TODO: Tune this value
    public static final double kDriveDerivativeGain = 0.0; // TODO: Tune this value
    public static final double kDriveStaticGain = 0.0; // TODO: Tune this value
    public static final double kDriveVelocityGain = 0.12; // TODO: Tune this value

    // ─── Steer Motor PID Gains ───────────────────────────────────────────────
    public static final double kSteerProportionalGain = 100.0; // TODO: Tune this value
    public static final double kSteerIntegralGain = 0.0; // TODO: Tune this value
    public static final double kSteerDerivativeGain = 0.5; // TODO: Tune this value

    // ─── Slip Current ────────────────────────────────────────────────────────
    public static final double kSlipCurrentAmps = 80.0; // TODO: Tune this value

    // ─── Front Left Module ───────────────────────────────────────────────────
    public static final int kFrontLeftDriveMotorId = 1; // TODO: Set to actual CAN ID
    public static final int kFrontLeftSteerMotorId = 2; // TODO: Set to actual CAN ID
    public static final int kFrontLeftCANcoderId = 3; // TODO: Set to actual CAN ID
    public static final double kFrontLeftCANcoderOffsetRotations = 0.0; // TODO: Set to actual offset (rotations)
    public static final boolean kFrontLeftDriveInverted = false; // TODO: Set based on module orientation

    // ─── Front Right Module ──────────────────────────────────────────────────
    public static final int kFrontRightDriveMotorId = 4; // TODO: Set to actual CAN ID
    public static final int kFrontRightSteerMotorId = 5; // TODO: Set to actual CAN ID
    public static final int kFrontRightCANcoderId = 6; // TODO: Set to actual CAN ID
    public static final double kFrontRightCANcoderOffsetRotations = 0.0; // TODO: Set to actual offset (rotations)
    public static final boolean kFrontRightDriveInverted = false; // TODO: Set based on module orientation

    // ─── Back Left Module ────────────────────────────────────────────────────
    public static final int kBackLeftDriveMotorId = 7; // TODO: Set to actual CAN ID
    public static final int kBackLeftSteerMotorId = 8; // TODO: Set to actual CAN ID
    public static final int kBackLeftCANcoderId = 9; // TODO: Set to actual CAN ID
    public static final double kBackLeftCANcoderOffsetRotations = 0.0; // TODO: Set to actual offset (rotations)
    public static final boolean kBackLeftDriveInverted = false; // TODO: Set based on module orientation

    // ─── Back Right Module ───────────────────────────────────────────────────
    public static final int kBackRightDriveMotorId = 13; // TODO: Set to actual CAN ID
    public static final int kBackRightSteerMotorId = 11; // TODO: Set to actual CAN ID
    public static final int kBackRightCANcoderId = 12; // TODO: Set to actual CAN ID
    public static final double kBackRightCANcoderOffsetRotations = 0.0; // TODO: Set to actual offset (rotations)
    public static final boolean kBackRightDriveInverted = false; // TODO: Set based on module orientation
  }

  public static final class SuperstructureConstants {
    // Arm Positions for Superstructure States
    public static final double kStowAngleRadians = Math.toRadians(0.0); // TODO: Tune this value
    public static final double kIntakeGroundAngleRadians = Math.toRadians(-45.0); // TODO: Tune this value
    public static final double kIntakeSourceAngleRadians = Math.toRadians(30.0); // TODO: Tune this value
    public static final double kShootAngleRadians = Math.toRadians(60.0); // TODO: Tune this value
    public static final double kScoreLowAngleRadians = Math.toRadians(45.0); // TODO: Tune this value
    public static final double kScoreHighAngleRadians = Math.toRadians(75.0); // TODO: Tune this value
    public static final double kClimbAngleRadians = Math.toRadians(0.0); // TODO: Tune this value

    // Sequencer Positions for Superstructure States
    public static final double kStowHeightMeters = 0.00; // TODO: Tune this value
    public static final double kIntakeGroundHeightMeters = 0.10; // TODO: Tune this value
    public static final double kIntakeSourceHeightMeters = 0.60; // TODO: Tune this value
    public static final double kShootHeightMeters = 0.80; // TODO: Tune this value
    public static final double kScoreLowHeightMeters = 0.30; // TODO: Tune this value
    public static final double kScoreHighHeightMeters = 1.10; // TODO: Tune this value
    public static final double kClimbHeightMeters = 1.20; // TODO: Tune this value
  }
}
