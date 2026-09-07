// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sequencer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SequencerConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Sequencer subsystem — single-mode spinning feeder motor controlled via kV feedforward.
 */
public class Sequencer extends SubsystemBase {
  private final SequencerIO m_sequencerIO;
  private final SequencerIOInputsAutoLogged m_inputs = new SequencerIOInputsAutoLogged();

  private double m_targetVelocityRotationsPerSecond = 0.0;

  /**
   * Creates a new Sequencer subsystem backed by the provided IO implementation.
   *
   * @param sequencerIO the hardware abstraction layer to use
   */
  public Sequencer(SequencerIO sequencerIO) {
    m_sequencerIO = sequencerIO;
  }

  // ─── Feed Control ──────────────────────────────────────────────────────────

  /** Spins the sequencer forward to feed all balls into the shooter using kV feedforward. */
  public void feed() {
    setVelocity(SequencerConstants.kFeedVelocityRotationsPerSecond);
  }

  /**
   * Sets closed-loop target velocity in rotations per second using kV feedforward.
   *
   * @param velocityRotationsPerSecond target velocity
   */
  public void setVelocity(double velocityRotationsPerSecond) {
    m_targetVelocityRotationsPerSecond = velocityRotationsPerSecond;
    m_sequencerIO.setVelocity(velocityRotationsPerSecond);
  }

  /** Stops the sequencer motor. */
  public void stop() {
    m_targetVelocityRotationsPerSecond = 0.0;
    m_sequencerIO.stop();
  }

  /** Sets raw voltage to the sequencer motor. */
  public void setVoltage(double appliedVolts) {
    m_targetVelocityRotationsPerSecond = 0.0;
    m_sequencerIO.setVoltage(appliedVolts);
  }

  // ─── Status & Getters ──────────────────────────────────────────────────────

  /** @return current sequencer velocity in rotations per second */
  public double getVelocityRotationsPerSecond() {
    return m_inputs.velocityRotationsPerSecond;
  }

  /** @return target sequencer velocity in rotations per second */
  public double getTargetVelocityRotationsPerSecond() {
    return m_targetVelocityRotationsPerSecond;
  }

  /** @return true if the sequencer is at target feeding velocity within tolerance */
  public boolean isAtTargetSpeed() {
    if (m_targetVelocityRotationsPerSecond <= 0.0) {
      return false;
    }
    return Math.abs(m_inputs.velocityRotationsPerSecond - m_targetVelocityRotationsPerSecond)
        <= SequencerConstants.kToleranceRotationsPerSecond;
  }

  /** @return current drawn by the leader motor in amps */
  public double getLeaderCurrentAmps() {
    return m_inputs.leaderCurrentAmps;
  }

  /** @return current drawn by the follower motor in amps */
  public double getFollowerCurrentAmps() {
    return m_inputs.followerCurrentAmps;
  }

  /** @return total current drawn by the sequencer in amps */
  public double getCurrentAmps() {
    return m_inputs.currentAmps;
  }

  /** @return applied voltage to the sequencer in volts */
  public double getAppliedVolts() {
    return m_inputs.appliedVolts;
  }

  // ─── Command Factories ─────────────────────────────────────────────────────

  /** @return command that continuously feeds balls */
  public Command feedCommand() {
    return Commands.startEnd(this::feed, this::stop, this).withName("Sequencer.feed");
  }

  /** @return command that stops the sequencer */
  public Command stopCommand() {
    return Commands.runOnce(this::stop, this).withName("Sequencer.stop");
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    m_sequencerIO.updateInputs(m_inputs);
    Logger.processInputs("Sequencer", m_inputs);

    Logger.recordOutput("Sequencer/VelocityRps", m_inputs.velocityRotationsPerSecond);
    Logger.recordOutput("Sequencer/PositionRotations", m_inputs.positionRotations);
    Logger.recordOutput("Sequencer/TargetVelocityRps", m_targetVelocityRotationsPerSecond);
    Logger.recordOutput("Sequencer/AppliedOutputVolts", m_inputs.appliedVolts);
    Logger.recordOutput("Sequencer/CurrentAmps", m_inputs.currentAmps);
    Logger.recordOutput("Sequencer/LeaderCurrentAmps", m_inputs.leaderCurrentAmps);
    Logger.recordOutput("Sequencer/FollowerCurrentAmps", m_inputs.followerCurrentAmps);
    Logger.recordOutput("Sequencer/AtTargetSpeed", isAtTargetSpeed());

    SmartDashboard.putNumber("Sequencer/Velocity (RPS)", m_inputs.velocityRotationsPerSecond);
    SmartDashboard.putNumber("Sequencer/Target Velocity (RPS)", m_targetVelocityRotationsPerSecond);
    SmartDashboard.putNumber("Sequencer/Applied Output (V)", m_inputs.appliedVolts);
    SmartDashboard.putNumber("Sequencer/Current (A)", m_inputs.currentAmps);
    SmartDashboard.putNumber("Sequencer/Leader Current (A)", m_inputs.leaderCurrentAmps);
    SmartDashboard.putNumber("Sequencer/Follower Current (A)", m_inputs.followerCurrentAmps);
    SmartDashboard.putBoolean("Sequencer/At Target Speed", isAtTargetSpeed());
  }
}
