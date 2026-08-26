// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.superstructure.SuperstructureState.EndEffectorAction;

/**
 * The Superstructure coordinates the Elevator, Arm, and EndEffector subsystems
 * using an enum-based state machine.
 *
 * <p>Typical usage in {@link frc.robot.RobotContainer}:
 * <pre>
 *   m_superstructure.setStateCommand(SuperstructureState.SCORE_HIGH).schedule();
 * </pre>
 *
 * <p>Transition safety notes:
 * <ul>
 *   <li>The elevator moves first when going UP (arm could collide with structure if it
 *       extends at a low height — adjust sequencing for your robot geometry).</li>
 *   <li>The arm moves first when going DOWN (retract arm before lowering elevator).</li>
 * </ul>
 * Customize {@link #applyState(SuperstructureState)} to match your robot's collision zones.
 */
public class Superstructure extends SubsystemBase {

  // ─── Subsystems ────────────────────────────────────────────────────────────

  private final Elevator    m_elevator;
  private final Arm         m_arm;
  private final EndEffector m_endEffector;

  // ─── State machine ─────────────────────────────────────────────────────────

  private SuperstructureState m_currentState  = SuperstructureState.STOW;
  private SuperstructureState m_desiredState  = SuperstructureState.STOW;

  // ─── Constructor ───────────────────────────────────────────────────────────

  public Superstructure(Elevator elevator, Arm arm, EndEffector endEffector) {
    m_elevator    = elevator;
    m_arm         = arm;
    m_endEffector = endEffector;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Returns a {@link Command} that transitions the superstructure to {@code state}
   * and finishes once both elevator and arm are at their goals.
   *
   * @param state the desired {@link SuperstructureState}
   */
  public Command setStateCommand(SuperstructureState state) {
    return Commands.runOnce(() -> {
          m_desiredState = state;
          applyState(state);
        }, this)
        .andThen(Commands.waitUntil(this::atDesiredState))
        .withName("Superstructure → " + state.name());
  }

  /**
   * Returns a {@link Command} that holds the superstructure at {@code state}
   * indefinitely (until cancelled). Useful for teleop triggers.
   *
   * @param state the desired {@link SuperstructureState}
   */
  public Command holdStateCommand(SuperstructureState state) {
    return Commands.run(() -> {
          m_desiredState = state;
          applyState(state);
        }, this)
        .withName("Hold → " + state.name());
  }

  /** @return the state the superstructure is currently transitioning toward */
  public SuperstructureState getDesiredState() {
    return m_desiredState;
  }

  /** @return the last fully-reached state */
  public SuperstructureState getCurrentState() {
    return m_currentState;
  }

  /** @return true when elevator and arm have both reached the desired state goals */
  public boolean atDesiredState() {
    return m_elevator.atGoal() && m_arm.atGoal();
  }

  // ─── State application ─────────────────────────────────────────────────────

  /**
   * Applies the setpoints for a given state to the subsystems.
   *
   * <p>Override this method to add collision-avoidance sequencing specific to
   * your robot's geometry (e.g. delay arm movement until elevator is above a
   * certain height).
   */
  private void applyState(SuperstructureState state) {
    m_elevator.setGoal(state.elevatorHeightMeters);
    m_arm.setGoal(state.armAngleRadians);
    applyEndEffectorAction(state.endEffectorAction);
  }

  private void applyEndEffectorAction(EndEffectorAction action) {
    switch (action) {
      case INTAKE  -> m_endEffector.intake();
      case OUTTAKE -> m_endEffector.outtake();
      default      -> m_endEffector.stop();
    }
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
