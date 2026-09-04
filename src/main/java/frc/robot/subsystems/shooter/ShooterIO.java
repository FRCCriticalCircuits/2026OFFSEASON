// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/**
 * Hardware abstraction interface for the Shooter subsystem (Flywheel + Adjustable Hood).
 */
public interface ShooterIO {

  /** Holds a snapshot of all sensor readings and control states from the shooter hardware. */
  class ShooterIOInputs {
    // ── Flywheel ────────────────────────────────────────────────────────────
    /** Current flywheel rotational velocity in rotations per second. */
    public double flywheelVelocityRotationsPerSecond = 0.0;

    /** Desired/target flywheel velocity in rotations per second. */
    public double flywheelTargetVelocityRotationsPerSecond = 0.0;

    /** Applied voltage to the leader flywheel motor (volts). */
    public double flywheelAppliedVolts = 0.0;

    /** Stator current drawn by the leader flywheel motor (amps). */
    public double flywheelLeaderCurrentAmps = 0.0;

    /** Stator current drawn by the follower 1 flywheel motor (amps). */
    public double flywheelFollower1CurrentAmps = 0.0;
    /** Alias for backward compatibility. */
    public double flywheelFollowerCurrentAmps = 0.0;

    /** Stator current drawn by the follower 2 flywheel motor (amps). */
    public double flywheelFollower2CurrentAmps = 0.0;

    /** Stator current drawn by the follower 3 flywheel motor (amps). */
    public double flywheelFollower3CurrentAmps = 0.0;

    /** Stator current drawn by the follower 4 flywheel motor (amps). */
    public double flywheelFollower4CurrentAmps = 0.0;

    // ── Hood ────────────────────────────────────────────────────────────────
    /** Current hood angle in radians. */
    public double hoodAngleRadians = 0.0;

    /** Desired/target hood angle in radians. */
    public double hoodTargetAngleRadians = 0.0;

    /** Applied voltage to the hood motor (volts). */
    public double hoodAppliedVolts = 0.0;

    /** Stator current drawn by the hood motor (amps). */
    public double hoodCurrentAmps = 0.0;
  }

  /**
   * Refreshes {@code inputs} with the latest hardware readings.
   *
   * @param inputs the struct to update in-place
   */
  default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Commands the flywheel to closed-loop velocity setpoint.
   *
   * @param velocityRotationsPerSecond target velocity in rotations per second
   */
  default void setFlywheelVelocity(double velocityRotationsPerSecond) {}

  /**
   * Commands the flywheel motors with raw voltage (open-loop).
   *
   * @param appliedVolts voltage to apply (-12.0 to 12.0)
   */
  default void setFlywheelVoltage(double appliedVolts) {}

  /** Stops the flywheel. */
  default void stopFlywheel() {}


  /**
   * Commands the hood motor to target angle in radians.
   *
   * @param angleRadians target angle in radians
   */
  default void setHoodAngle(double angleRadians) {}

  /**
   * Commands the hood motor with raw voltage.
   *
   * @param appliedVolts voltage to apply (-12.0 to 12.0)
   */
  default void setHoodVoltage(double appliedVolts) {}

  /** Stops the hood motor. */
  default void stopHood() {}

  /** Resets the hood encoder position to 0.0. */
  default void resetHoodEncoder() {}
}
