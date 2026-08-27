// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sequencer;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SequencerConstants;

/**
 * Sequencer subsystem.
 *
 * <p>All hardware I/O is delegated to a {@link SequencerIO} implementation injected
 * at construction time. The subsystem itself only contains control logic
 * (ProfiledPID + ElevatorFeedforward), making it trivially testable with any IO
 * backend (simulation, replay, or real hardware).
 */
public class Sequencer extends SubsystemBase {

  // ─── IO layer ─────────────────────────────────────────────────────────────

  private final SequencerIO m_sequencerIO;
  private final SequencerIO.SequencerIOInputs m_inputs = new SequencerIO.SequencerIOInputs();

  // ─── Controllers ──────────────────────────────────────────────────────────

  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          SequencerConstants.kStaticGain,
          SequencerConstants.kGravityGain,
          SequencerConstants.kVelocityGain,
          SequencerConstants.kAccelerationGain);

  private final ProfiledPIDController m_feedbackController =
      new ProfiledPIDController(
          SequencerConstants.kProportionalGain,
          SequencerConstants.kIntegralGain,
          SequencerConstants.kDerivativeGain,
          new TrapezoidProfile.Constraints(
              SequencerConstants.kMaxVelocityMetersPerSecond,
              SequencerConstants.kMaxAccelerationMetersPerSecondSquared));

  // ─── State ────────────────────────────────────────────────────────────────

  private double m_goalHeightMeters = 0.0;

  // ─── Constructor ──────────────────────────────────────────────────────────

  /**
   * Creates a new Sequencer subsystem backed by the provided IO implementation.
   *
   * @param sequencerIO the hardware abstraction layer to use
   */
  public Sequencer(SequencerIO sequencerIO) {
    m_sequencerIO = sequencerIO;
    m_feedbackController.setTolerance(SequencerConstants.kToleranceMeters);
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Sets the desired height goal for the sequencer.
   *
   * @param targetHeightMeters target height in meters
   */
  public void setGoal(double targetHeightMeters) {
    m_goalHeightMeters = targetHeightMeters;
    m_feedbackController.setGoal(targetHeightMeters);
  }

  /** @return true when the sequencer is at the goal height within tolerance */
  public boolean atGoal() {
    return m_feedbackController.atGoal();
  }

  /** @return current sequencer height in meters (sourced from IO inputs) */
  public double getHeightMeters() {
    return m_inputs.heightMeters;
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // 1. Refresh sensor snapshot from hardware / sim
    m_sequencerIO.updateInputs(m_inputs);

    // 2. Auto-zero when the lower limit switch is tripped
    if (m_inputs.lowerLimitSwitchTripped) {
      m_feedbackController.reset(0.0);
      m_sequencerIO.resetEncoder();
    }

    // 3. Calculate control output
    double feedbackOutputVolts = m_feedbackController.calculate(m_inputs.heightMeters);
    double feedforwardOutputVolts =
        m_feedforward.calculate(m_feedbackController.getSetpoint().velocity);
    double totalAppliedVolts = feedbackOutputVolts + feedforwardOutputVolts;

    // 4. Send voltage command to hardware / sim
    m_sequencerIO.setVoltage(totalAppliedVolts);

    // 5. Telemetry
    SmartDashboard.putNumber("Sequencer/Height (m)", m_inputs.heightMeters);
    SmartDashboard.putNumber("Sequencer/Velocity (m per sec)", m_inputs.velocityMetersPerSecond);
    SmartDashboard.putNumber("Sequencer/Goal (m)", m_goalHeightMeters);
    SmartDashboard.putNumber("Sequencer/Applied Output (V)", totalAppliedVolts);
    SmartDashboard.putNumber("Sequencer/Current (A)", m_inputs.currentAmps);
    SmartDashboard.putBoolean("Sequencer/At Goal", atGoal());
    SmartDashboard.putBoolean("Sequencer/Lower Limit Switch", m_inputs.lowerLimitSwitchTripped);
    SmartDashboard.putBoolean("Sequencer/Upper Limit Switch", m_inputs.upperLimitSwitchTripped);
  }
}
