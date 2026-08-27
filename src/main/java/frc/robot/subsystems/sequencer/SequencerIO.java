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
 *
 * <p>Implementations:
 * <ul>
 *   <li>{@link SequencerIOSim}     — WPILib simulation
 *   <li>{@link SequencerIOKraken}  — real CTRE Kraken X60 (TalonFX) hardware
 * </ul>
 */
public interface SequencerIO {

  // ─── Inputs (hardware → subsystem) ────────────────────────────────────────

  /**
   * Holds a snapshot of all sensor readings from the sequencer hardware.
   * The subsystem calls {@link SequencerIO#updateInputs(SequencerIOInputs)} once
   * per periodic loop to refresh this struct.
   */
  class SequencerIOInputs {
    /** Current sequencer height in meters (0 = fully retracted). */
    public double heightMeters = 0.0;

    /** Current sequencer velocity in meters per second. */
    public double velocityMetersPerSecond = 0.0;

    /** Applied voltage to the sequencer motor(s) (volts). */
    public double appliedVolts = 0.0;

    /** Supply current drawn by the sequencer motor(s) (amps). */
    public double currentAmps = 0.0;

    /** Whether the sequencer bottom (lower) limit switch is tripped. */
    public boolean lowerLimitSwitchTripped = false;

    /** Whether the sequencer top (upper) limit switch is tripped. */
    public boolean upperLimitSwitchTripped = false;
  }

  // ─── Default no-op implementations ────────────────────────────────────────
  // Allows partial implementations; unused methods don't need to be overridden.

  /**
   * Refreshes {@code inputs} with the latest hardware readings.
   *
   * <p>Must be called once per periodic loop <em>before</em> any control
   * output is calculated.
   *
   * @param inputs the struct to update in-place
   */
  default void updateInputs(SequencerIOInputs inputs) {}

  /**
   * Commands the sequencer motor(s) to output the given voltage.
   *
   * @param appliedVolts voltage to apply (positive = upward)
   */
  default void setVoltage(double appliedVolts) {}

  /**
   * Zeros the sequencer encoder at the current mechanical position.
   * Call this when the sequencer is confirmed to be at the bottom hard-stop.
   */
  default void resetEncoder() {}

  /**
   * Configures the brake/coast idle mode of the sequencer motor(s).
   *
   * @param enableBrakeMode {@code true} = brake mode, {@code false} = coast mode
   */
  default void setBrakeMode(boolean enableBrakeMode) {}
}
