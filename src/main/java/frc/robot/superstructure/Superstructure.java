// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.sequencer.Sequencer;

/**
 * The Superstructure coordinates the Sequencer and Arm subsystems
 * using an enum-based state machine.
 *
 * <p>Typical usage in {@link frc.robot.RobotContainer}:
 * <pre>
 *   m_superstructure.setStateCommand(SuperstructureState.SCORE_HIGH).schedule();
 * </pre>
 *
 * <p>Transition safety notes:
 * <ul>
 *   <li>The sequencer moves first when going UP (arm could collide with structure if it
 *       extends at a low height — adjust sequencing for your robot geometry).</li>
 *   <li>The arm moves first when going DOWN (retract arm before lowering sequencer).</li>
 * </ul>
 * Customize {@link #applyState(SuperstructureState)} to match your robot's collision zones.
 */
public class Superstructure extends SubsystemBase {

  // ─── Subsystems ────────────────────────────────────────────────────────────

  private final Sequencer m_sequencer;
  private final Arm       m_arm;

  // ─── State machine ─────────────────────────────────────────────────────────

  private SuperstructureState m_currentState = SuperstructureState.STOW;
  private SuperstructureState m_desiredState  = SuperstructureState.STOW;

  // ─── Constructor ───────────────────────────────────────────────────────────

  public Superstructure(Sequencer sequencer, Arm arm) {
    m_sequencer = sequencer;
    m_arm       = arm;
  }

  // ─── Public API ────────────────────────────────────────────────────

  /**
   * Returns a {@link Command} that transitions the superstructure to {@code targetState}
   * and finishes once both sequencer and arm are at their goals.
   *
   * @param targetState the desired {@link SuperstructureState}
   */
  public Command setStateCommand(SuperstructureState targetState) {
    return Commands.runOnce(() -> {
          m_desiredState = targetState;
          applyState(targetState);
        }, this)
        .andThen(Commands.waitUntil(this::atDesiredState))
        .withName("Superstructure → " + targetState.name());
  }

  /**
   * Returns a {@link Command} that holds the superstructure at {@code targetState}
   * indefinitely (until cancelled). Useful for teleop triggers.
   *
   * @param targetState the desired {@link SuperstructureState}
   */
  public Command holdStateCommand(SuperstructureState targetState) {
    return Commands.run(() -> {
          m_desiredState = targetState;
          applyState(targetState);
        }, this)
        .withName("Hold → " + targetState.name());
  }

  /** @return the state the superstructure is currently transitioning toward */
  public SuperstructureState getDesiredState() {
    return m_desiredState;
  }

  /** @return the last fully-reached state */
  public SuperstructureState getCurrentState() {
    return m_currentState;
  }

  /** @return true when sequencer and arm have both reached the desired state goals */
  public boolean atDesiredState() {
    return m_sequencer.atGoal() && m_arm.atGoal();
  }

  // ─── State application ─────────────────────────────────────────────────────

  /**
   * Applies the setpoints for a given state to the subsystems.
   *
   * <p>Override this method to add collision-avoidance sequencing specific to
   * your robot's geometry (e.g. delay arm movement until sequencer is above a
   * certain height).
   */
  private void applyState(SuperstructureState targetState) {
    m_sequencer.setGoal(targetState.sequencerHeightMeters);
    m_arm.setGoal(targetState.armAngleRadians);
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // Promote desired → current when we arrive
    if (atDesiredState()) {
      m_currentState = m_desiredState;
    }

    SmartDashboard.putString("Superstructure/Current State", m_currentState.name());
    SmartDashboard.putString("Superstructure/Desired State", m_desiredState.name());
    SmartDashboard.putBoolean("Superstructure/At Goal",      atDesiredState());
  }
}
