// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

  // TODO: Replace with real motor controller (e.g. TalonFX, SparkMax)
  // private final TalonFX m_motor = new TalonFX(EndEffectorConstants.kMotorId);

  // TODO: Replace with real sensor (e.g. DigitalInput, AnalogInput, CANrange)
  // private final DigitalInput m_beamBreak = new DigitalInput(EndEffectorConstants.kBeamBreakPort);

  /** Intake speed (positive = intaking). Tune as needed. */
  private static final double kIntakeSpeed  =  0.8; // volts or duty cycle
  /** Outtake / score speed (negative = ejecting). */
  private static final double kOuttakeSpeed = -0.8;

  private double m_appliedOutput = 0.0;

  public EndEffector() {}

  // ─── Public API ────────────────────────────────────────────────────────────

  /** Runs the intake rollers to collect a game piece. */
  public void intake() {
    setOutput(kIntakeSpeed);
  }

  /** Runs the rollers in reverse to score / eject a game piece. */
  public void outtake() {
    setOutput(kOuttakeSpeed);
  }

  /** Stops the end effector rollers. */
  public void stop() {
    setOutput(0.0);
  }

  /**
   * @return true if the end effector is holding a game piece.
   *         Replace with real sensor logic.
   */
  public boolean hasGamePiece() {
    // TODO: return real sensor value, e.g.:
    // return !m_beamBreak.get(); // beam-break is normally-closed
    return false;
  }

  // ─── Private Helpers ───────────────────────────────────────────────────────

  private void setOutput(double output) {
    m_appliedOutput = output;
    // TODO: apply to real motor, e.g.:
    // m_motor.set(output);
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    SmartDashboard.putNumber("EndEffector/Output",       m_appliedOutput);
    SmartDashboard.putBoolean("EndEffector/Has Piece",   hasGamePiece());
  }
}
