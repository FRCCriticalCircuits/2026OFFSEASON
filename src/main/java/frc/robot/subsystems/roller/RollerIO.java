// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

/**
 * Hardware abstraction interface for the Roller (intake) subsystem.
 *
 * <p>All hardware interaction is routed through this interface so that the
 * {@link Roller} subsystem logic is fully decoupled from any specific motor
 * controller or simulation backend.
 */
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  /** Holds a snapshot of all sensor readings from the roller hardware. */
  @AutoLog
  class RollerIOInputs {
    /** Current roller rotational velocity in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;

    /** Applied voltage to the leader roller motor (volts). */
    public double appliedVolts = 0.0;

    /** Stator current drawn by the leader roller motor (amps). */
    public double leaderCurrentAmps = 0.0;

    /** Stator current drawn by the follower roller motor (amps). */
    public double followerCurrentAmps = 0.0;

    /** Stator current drawn by the roller leader motor (amps) for backwards compatibility. */
    public double currentAmps = 0.0;

    /** Whether a game piece / ball is detected inside the intake roller. */
    public boolean gamePieceDetected = false;
  }

  /**
   * Refreshes {@code inputs} with the latest hardware readings.
   *
   * @param inputs the struct to update in-place
   */
  default void updateInputs(RollerIOInputs inputs) {}

  /**
   * Commands the roller motor to output the given voltage.
   *
   * @param appliedVolts voltage to apply (positive = intake, negative = outtake)
   */
  default void setVoltage(double appliedVolts) {}
  
  /**
   * Configures the brake/coast idle mode of the roller motor.
   *
   * @param enableBrakeMode {@code true} = brake mode, {@code false} = coast mode
   */
  default void setBrakeMode(boolean enableBrakeMode) {}
}
