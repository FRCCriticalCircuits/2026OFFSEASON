// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static final). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or its individual members) to use them.
 */
public final class Constants {

  /** CAN bus name for all CTRE devices ("" for default RIO bus or "canivore"). */
  public static final String kCANBusName = "";

  public static final class DriverConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kTriggerThreshold = 0.25;
  }

  public static final class ArmConstants {
    // CAN ID (Intake Pivot / Deploy Motor)
    public static final int kMotorId = 30;

    // Physical Constants
    public static final double kGearRatio = 50.0;
    public static final double kArmLengthMeters = Units.inchesToMeters(17.7); // 0.45 m
    public static final double kArmMassKilograms = 3.0;
    public static final double kMinAngleRadians = Math.toRadians(-90.0);
    public static final double kMaxAngleRadians = Math.toRadians(90.0);

    // Kraken X60 Current Limits
    public static final double kStatorCurrentLimitAmps = 60.0;
    public static final double kSupplyCurrentLimitAmps = 40.0;

    // Control Gains (PID + Feedforward)
    public static final double kProportionalGain = 25.0;
    public static final double kIntegralGain = 0.0;
    public static final double kDerivativeGain = 0.0;
    public static final double kStaticGain = 0.0;
    public static final double kGravityGain = 0.0;
    public static final double kVelocityGain = 0.0;
    public static final double kAccelerationGain = 0.0;

    // Profile Constraints
    public static final double kMaxVelocityRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxAccelerationRadiansPerSecondSquared = Math.PI * 4.0;
    public static final double kToleranceRadians = Math.toRadians(2.0);
  }

  public static final class RollerConstants {
    // CAN ID (Intake Roller Leader)
    public static final int kMotorId = 31;

    // Physical Constants
    public static final double kGearRatio = 1.0;

    // Kraken X60 Current Limits
    public static final double kStatorCurrentLimitAmps = 60.0;
    public static final double kSupplyCurrentLimitAmps = 60.0;

    // Applied Voltages for Roller Actions
    public static final double kIntakeAppliedVolts = 8.0;
    public static final double kEjectAppliedVolts = -8.0;
    public static final double kHoldAppliedVolts = 2.0;
  }

  public static final class SequencerConstants {
    // CAN ID (Feeder Motor)
    public static final int kMotorId = 34;

    // Physical Constants
    public static final double kGearRatio = 1.0;

    // Kraken X60 Current Limits
    public static final double kStatorCurrentLimitAmps = 80.0;
    public static final double kSupplyCurrentLimitAmps = 50.0;

    // Velocity Feedforward Control Gain (kV) & Feedback
    public static final double kVelocityGain = 0.12; // Volts per RPS
    public static final double kStaticGain = 0.25;   // Volts
    public static final double kProportionalGain = 0.1;

    // Operating Setpoint (Rotations per second)
    public static final double kFeedVelocityRotationsPerSecond = 50.0; // ~3000 RPM feed speed
    public static final double kToleranceRotationsPerSecond = 2.5;
  }

  public static final class ShooterConstants {
    // ─── Flywheel Motors (Kraken X60) ────────────────────────────────────────
    public static final int kFlywheelLeaderMotorId = 35;
    public static final int kFlywheelFollowerMotorId = 36;
    public static final double kFlywheelGearRatio = 1.0;

    public static final double kFlywheelStatorCurrentLimitAmps = 80.0;
    public static final double kFlywheelSupplyCurrentLimitAmps = 60.0;

    // Flywheel Velocity Closed-Loop PID & Feedforward Gains (Slot 0 on Kraken X60 TalonFX)
    public static final double kFlywheelProportionalGain = 0.15;
    public static final double kFlywheelIntegralGain = 0.0;
    public static final double kFlywheelDerivativeGain = 0.0;
    public static final double kFlywheelStaticGain = 0.25;
    public static final double kFlywheelVelocityGain = 0.12;
    public static final double kFlywheelAccelerationGain = 0.01;

    // Flywheel Target Velocity Setpoints (rotations per second)
    public static final double kFlywheelTargetVelocityRotationsPerSecond = 70.0; // ~4200 RPM
    public static final double kFlywheelIdleVelocityRotationsPerSecond = 20.0; // ~1200 RPM idle spool
    public static final double kFlywheelToleranceRotationsPerSecond = 2.5; // (150 RPM)

    // ─── Hood Motor (Kraken X44 / X60) ───────────────────────────────────────
    public static final int kHoodMotorId = 37;
    public static final double kHoodGearRatio = 1.0;

    public static final double kHoodMinAngleRadians = Math.toRadians(0.0);
    public static final double kHoodMaxAngleRadians = Math.toRadians(60.0);

    public static final double kHoodStatorCurrentLimitAmps = 40.0;
    public static final double kHoodSupplyCurrentLimitAmps = 40.0;

    // Hood Position Closed-Loop PID Gains (Slot 0 on Kraken X44/X60 TalonFX)
    public static final double kHoodProportionalGain = 20.0;
    public static final double kHoodIntegralGain = 0.0;
    public static final double kHoodDerivativeGain = 0.0;
    public static final double kHoodToleranceRadians = Math.toRadians(1.0);

    // Hood Angle Preset Targets
    public static final double kHoodStowAngleRadians = Math.toRadians(0.0);
  }

  public static final class AutoAimConstants {
    // ─── Field Goal Locations (2026 REBUILT Field Coordinates) ───────────────
    // Field dimensions: 651.22 in x 317.69 in. Hub distance from alliance wall: 182.11 in.
    public static final Translation2d kBlueGoalLocation =
        new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(317.69) / 2.0);
    public static final Translation2d kRedGoalLocation =
        new Translation2d(
            Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(317.69) / 2.0);

    // ─── Drivetrain Heading Alignment PID ────────────────────────────────────
    public static final double kHeadingProportionalGain = 9.0;
    public static final double kHeadingIntegralGain = 0.0;
    public static final double kHeadingDerivativeGain = 0.15;
    public static final double kHeadingToleranceRadians = Math.toRadians(2.0);

    // Shooter exits off the back of the chassis (180 deg offset)
    public static final Rotation2d kTargetAimOffset = Rotation2d.k180deg;

    // ─── Distance Boundaries ─────────────────────────────────────────────────
    public static final double kMinDistanceMeters = 1.0;
    public static final double kMaxDistanceMeters = 7.0;

    // ─── Safety Speed Reduction ──────────────────────────────────────────────
    public static final double kAutoAimMaxSpeedMultiplier = 0.70; // 30% speed reduction during auto-aim
  }

  public static final class SwerveConstants {
    // ─── IMU ─────────────────────────────────────────────────────────────────
    public static final int kPigeon2CanId = 20;

    // ─── Robot Dimensions ────────────────────────────────────────────────────
    public static final double kTrackWidthMeters = Units.inchesToMeters(21.9); // 2 * 10.95 in = 0.55626 m
    public static final double kWheelbaseMeters = Units.inchesToMeters(21.9);  // 2 * 10.95 in = 0.55626 m

    // ─── Module Gear Ratios ──────────────────────────────────────────────────
    public static final double kDriveGearRatio = 6.026785714285714;
    public static final double kSteerGearRatio = 26.09090909090909;

    // ─── Wheel ───────────────────────────────────────────────────────────────
    public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0); // 0.0508 m
    public static final double kWheelCircumferenceMeters = 2.0 * Math.PI * kWheelRadiusMeters;

    // ─── Speed Limits ────────────────────────────────────────────────────────
    public static final double kMaxSpeedMetersPerSecond = 5.12;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2.0;

    // ─── Drive Motor Current Limits ──────────────────────────────────────────
    public static final double kDriveStatorCurrentLimitAmps = 120.0;
    public static final double kDriveSupplyCurrentLimitAmps = 60.0;

    // ─── Steer Motor Current Limits ──────────────────────────────────────────
    public static final double kSteerStatorCurrentLimitAmps = 60.0;
    public static final double kSteerSupplyCurrentLimitAmps = 40.0;

    // ─── Drive Motor PID Gains ───────────────────────────────────────────────
    public static final double kDriveProportionalGain = 0.05;
    public static final double kDriveIntegralGain = 0.0;
    public static final double kDriveDerivativeGain = 0.0;
    public static final double kDriveStaticGain = 0.0;
    public static final double kDriveVelocityGain = 0.12;

    // ─── Steer Motor PID Gains ───────────────────────────────────────────────
    public static final double kSteerProportionalGain = 59.5;
    public static final double kSteerIntegralGain = 0.0;
    public static final double kSteerDerivativeGain = 0.075;

    // ─── Slip Current ────────────────────────────────────────────────────────
    public static final double kSlipCurrentAmps = 120.0;

    // ─── Front Left Module ───────────────────────────────────────────────────
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 2;
    public static final int kFrontLeftCANcoderId = 3;
    public static final double kFrontLeftCANcoderOffsetRotations = -0.252197265625;
    public static final boolean kFrontLeftDriveInverted = false;

    // ─── Front Right Module ──────────────────────────────────────────────────
    public static final int kFrontRightDriveMotorId = 4;
    public static final int kFrontRightSteerMotorId = 5;
    public static final int kFrontRightCANcoderId = 6;
    public static final double kFrontRightCANcoderOffsetRotations = -0.48046875;
    public static final boolean kFrontRightDriveInverted = true;

    // ─── Back Left Module ────────────────────────────────────────────────────
    public static final int kBackLeftDriveMotorId = 7;
    public static final int kBackLeftSteerMotorId = 8;
    public static final int kBackLeftCANcoderId = 9;
    public static final double kBackLeftCANcoderOffsetRotations = 0.327880859375;
    public static final boolean kBackLeftDriveInverted = false;

    // ─── Back Right Module ───────────────────────────────────────────────────
    public static final int kBackRightDriveMotorId = 10;
    public static final int kBackRightSteerMotorId = 11;
    public static final int kBackRightCANcoderId = 12;
    public static final double kBackRightCANcoderOffsetRotations = -0.247314453125;
    public static final boolean kBackRightDriveInverted = true;

    // ─── Autonomous PID & Physical Constants ─────────────────────────────────
    public static final double kAutoTranslationKP = 5.0;
    public static final double kAutoTranslationKI = 0.0;
    public static final double kAutoTranslationKD = 0.0;

    public static final double kAutoRotationKP = 5.0;
    public static final double kAutoRotationKI = 0.0;
    public static final double kAutoRotationKD = 0.0;

    public static final double kRobotMassKg = 45.0;
    public static final double kRobotMOIKgM2 = 4.0;
    public static final double kWheelCOF = 1.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI * 4.0;

    public static final double kTranslationKP = 5.0;
    public static final double kTranslationKI = 0.0;
    public static final double kTranslationKD = 0.0;

    public static final double kRotationKP = 5.0;
    public static final double kRotationKI = 0.0;
    public static final double kRotationKD = 0.0;
  }

  public static final class SuperstructureConstants {
    // Arm Positions for Superstructure States
    public static final double kStowAngleRadians = Math.toRadians(0.0);
    public static final double kIntakeGroundAngleRadians = Units.degreesToRadians(-75.0);
    public static final double kShootAngleRadians = Math.toRadians(60.0);
  }
}
