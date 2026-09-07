// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sequencer;

/**
 * Hardware abstraction interface for the Sequencer subsystem.
 *
 * <p>All hardware interaction is routed through this interface so that the
 * {@link Sequencer} subsystem logic is fully decoupled from any specific motor
 * controller, encoder, or simulation backend.
 */
import org.littletonrobotics.junction.AutoLog;

public interface SequencerIO {

  // ─── Inputs (hardware → subsystem) ────────────────────────────────────────

  /**
   * Holds a snapshot of all sensor readings from the sequencer hardware.
   */
  @AutoLog
  class SequencerIOInputs {
    /** Current sequencer rotational velocity in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;

    /** Current accumulated position in rotations. */
    public double positionRotations = 0.0;

    /** Applied voltage to the sequencer motor (volts). */
    public double appliedVolts = 0.0;

    /** Current drawn by the sequencer motor(s) combined (amps). */
    public double currentAmps = 0.0;

    /** Current drawn by the leader motor (amps). */
    public double leaderCurrentAmps = 0.0;

    /** Current drawn by the follower motor (amps). */
    public double followerCurrentAmps = 0.0;
  }

  // ─── Default no-op implementations ────────────────────────────────────────

  /** Refreshes {@code inputs} with the latest hardware readings. */
  default void updateInputs(SequencerIOInputs inputs) {}

  /**
   * Commands the sequencer motor to a target velocity using kV feedforward.
   *
   * @param velocityRotationsPerSecond target speed in rotations per second
   */
  default void setVelocity(double velocityRotationsPerSecond) {}

  /**
   * Commands the sequencer motor to output the given voltage.
   *
   * @param appliedVolts voltage to apply
   */
  default void setVoltage(double appliedVolts) {}

  /** Stops the sequencer motor. */
  default void stop() {}

  /**
   * Configures the brake/coast idle mode of the sequencer motor.
   *
   * @param enableBrakeMode {@code true} = brake mode, {@code false} = coast mode
   */
  default void setBrakeMode(boolean enableBrakeMode) {}
}
