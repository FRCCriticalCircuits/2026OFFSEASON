// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.sequencer.Sequencer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.superstructure.SuperstructureState.RollerAction;
import frc.robot.superstructure.SuperstructureState.ShooterAction;

/**
 * The Superstructure coordinates the Sequencer, Arm, Roller, and combined Shooter (Flywheel + Hood)
 * subsystems using an enum-based state machine.
 */
public class Superstructure extends SubsystemBase {

  // ─── Subsystems ────────────────────────────────────────────────────────────

  private final Sequencer m_sequencer;
  private final Arm       m_arm;
  private final Roller    m_roller;
  private final Shooter   m_shooter;

  // ─── State machine ─────────────────────────────────────────────────────────

  private SuperstructureState m_currentState = SuperstructureState.STOW;
  private SuperstructureState m_desiredState  = SuperstructureState.STOW;

  // ─── Constructor ───────────────────────────────────────────────────────────

  public Superstructure(Sequencer sequencer, Arm arm, Roller roller, Shooter shooter) {
    m_sequencer = sequencer;
    m_arm       = arm;
    m_roller    = roller;
    m_shooter   = shooter;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Returns a {@link Command} that transitions the superstructure to {@code targetState}
   * and finishes once mechanisms reach their goals.
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

  /**
   * Returns an automated shoot sequence command:
   * 1. Positions arm & sequencer, sets hood angle, and spools flywheel (SPIN_UP_SHOOT).
   * 2. Waits until flywheel reaches speed, hood reaches angle, and arm/sequencer arrive.
   * 3. Transitions to SHOOT to feed balls through sequencer into the shooter.
   */
  public Command shootSequenceCommand() {
    return setStateCommand(SuperstructureState.SPIN_UP_SHOOT)
        .andThen(Commands.waitUntil(() -> m_shooter.isReadyToShoot() && atDesiredState()))
        .andThen(holdStateCommand(SuperstructureState.SHOOT))
        .withName("Superstructure.shootSequence");
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
    boolean mechanismsAtGoal = m_sequencer.atGoal() && m_arm.atGoal();
    if (m_desiredState == SuperstructureState.SHOOT || m_desiredState == SuperstructureState.SPIN_UP_SHOOT) {
      return mechanismsAtGoal && m_shooter.isReadyToShoot();
    }
    return mechanismsAtGoal;
  }

  // ─── State application ─────────────────────────────────────────────────────

  /**
   * Applies the setpoints for a given state to all subsystems.
   */
  private void applyState(SuperstructureState targetState) {
    m_sequencer.setGoal(targetState.sequencerHeightMeters);
    m_arm.setGoal(targetState.armAngleRadians);
    applyRollerAction(targetState.rollerAction);
    applyShooterAction(targetState.shooterAction);
  }

  private void applyRollerAction(RollerAction action) {
    switch (action) {
      case INTAKE  -> m_roller.runIntake();
      case OUTTAKE -> m_roller.runOuttake();
      case HOLD    -> m_roller.runHold();
      case STOP    -> m_roller.stop();
    }
  }

  private void applyShooterAction(ShooterAction action) {
    switch (action) {
      case SPIN_UP_HIGH, SHOOT_HIGH -> {
        m_shooter.runFlywheel();
        m_shooter.setHoodHighGoal();
      }
      case SPIN_UP_LOW, SHOOT_LOW -> {
        m_shooter.runFlywheel();
        m_shooter.setHoodLowGoal();
      }
      case IDLE -> {
        m_shooter.runIdleFlywheel();
        m_shooter.stowHood();
      }
      case STOP -> m_shooter.stop();
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
