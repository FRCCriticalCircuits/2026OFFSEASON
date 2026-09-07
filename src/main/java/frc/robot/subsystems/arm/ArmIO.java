// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/**
 * Hardware abstraction interface for the Arm subsystem.
 *
 * <p>All hardware interaction is routed through this interface so that the
 * {@link Arm} subsystem logic is fully decoupled from any specific motor
 * controller, encoder, or simulation backend.
 *
 * <p>Implementations:
 * <ul>
 *   <li>{@link ArmIOSim}     — WPILib simulation
 *   <li>{@link ArmIOKraken}  — real CTRE Kraken X60 (TalonFX) hardware
 * </ul>
 */
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  // ─── Inputs (hardware → subsystem) ────────────────────────────────────────

  /**
   * Holds a snapshot of all sensor readings from the arm hardware.
   * The subsystem calls {@link ArmIO#updateInputs(ArmIOInputs)} once per
   * periodic loop to refresh this struct.
   */
  @AutoLog
  class ArmIOInputs {
    /** Current arm angle in radians (0 = horizontal, positive = up). */
    public double angleRadians = 0.0;

    /** Current arm angular velocity in radians per second. */
    public double velocityRadiansPerSecond = 0.0;

    /** Applied voltage to the arm motor (volts). */
    public double appliedVolts = 0.0;

    /** Supply current drawn by the arm motor (amps). */
    public double currentAmps = 0.0;

    /** Whether the arm forward (positive) limit switch is tripped. */
    public boolean forwardLimitSwitchTripped = false;

    /** Whether the arm reverse (negative) limit switch is tripped. */
    public boolean reverseLimitSwitchTripped = false;
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
  default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Commands the arm motor to output the given voltage.
   *
   * @param appliedVolts voltage to apply (positive = toward forward limit)
   */
  default void setVoltage(double appliedVolts) {}

  /**
   * Zeros the arm encoder at the current mechanical position.
   * Call this when the arm is confirmed to be at a known hard-stop.
   */
  default void resetEncoder() {}

  /**
   * Configures the brake/coast idle mode of the arm motor.
   *
   * @param enableBrakeMode {@code true} = brake mode, {@code false} = coast mode
   */
  default void setBrakeMode(boolean enableBrakeMode) {}
}
