// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

/**
 * Roller subsystem for intaking and outtaking game pieces/balls.
 */
public class Roller extends SubsystemBase {
  private final RollerIO m_rollerIO;
  private final RollerIO.RollerIOInputs m_inputs = new RollerIO.RollerIOInputs();

  /**
   * Creates a new Roller subsystem.
   *
   * @param rollerIO the hardware abstraction layer to use
   */
  public Roller(RollerIO rollerIO) {
    m_rollerIO = rollerIO;
  }

  // ─── Control Methods ───────────────────────────────────────────────────────

  /** Runs the roller at full intaking speed. */
  public void runIntake() {
    setVoltage(RollerConstants.kIntakeAppliedVolts);
  }

  /** Runs the roller in reverse to eject/outtake game pieces. */
  public void runOuttake() {
    setVoltage(RollerConstants.kOuttakeAppliedVolts);
  }

  /** Runs the roller at low voltage to hold/retain a game piece. */
  public void runHold() {
    setVoltage(RollerConstants.kHoldAppliedVolts);
  }

  /** Stops the roller motor. */
  public void stop() {
    setVoltage(0.0);
  }

  /**
   * Sets custom voltage directly to the roller motor.
   *
   * @param appliedVolts voltage from -12.0 to 12.0
   */
  public void setVoltage(double appliedVolts) {
    m_rollerIO.setVoltage(appliedVolts);
  }

  // ─── Getters ───────────────────────────────────────────────────────────────

  /** @return current roller velocity in rotations per second */
  public double getVelocityRotationsPerSecond() {
    return m_inputs.velocityRotationsPerSecond;
  }

  /** @return true if a game piece / ball is detected */
  public boolean isGamePieceDetected() {
    return m_inputs.gamePieceDetected;
  }

  // ─── Command Factories ─────────────────────────────────────────────────────

  /** @return a command that runs the intake while scheduled and stops on finish */
  public Command intakeCommand() {
    return Commands.startEnd(this::runIntake, this::stop, this).withName("Roller.intake");
  }

  /** @return a command that runs outtake while scheduled and stops on finish */
  public Command outtakeCommand() {
    return Commands.startEnd(this::runOuttake, this::stop, this).withName("Roller.outtake");
  }

  /** @return a command that stops the roller */
  public Command stopCommand() {
    return Commands.runOnce(this::stop, this).withName("Roller.stop");
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    m_rollerIO.updateInputs(m_inputs);

    SmartDashboard.putNumber("Roller/Velocity (RPS)", m_inputs.velocityRotationsPerSecond);
    SmartDashboard.putNumber("Roller/Applied Output (V)", m_inputs.appliedVolts);
    SmartDashboard.putNumber("Roller/Current (A)", m_inputs.currentAmps);
    SmartDashboard.putBoolean("Roller/Game Piece Detected", m_inputs.gamePieceDetected);
  }
}
