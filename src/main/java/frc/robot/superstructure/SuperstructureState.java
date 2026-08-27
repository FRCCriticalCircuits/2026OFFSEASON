// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import frc.robot.Constants.SuperstructureConstants;

/**
 * Defines every named state the superstructure can be in.
 *
 * <p>Each state carries the target position for the arm (radians), along with
 * the desired actions for the roller, shooter, and sequencer feeder.
 */
public enum SuperstructureState {

  // ─── State             Arm (rad)                                    RollerAction          ShooterAction          SequencerAction ─────
  STOW               (SuperstructureConstants.kStowAngleRadians,          RollerAction.STOP,    ShooterAction.STOP,    SequencerAction.STOP),
  INTAKE_GROUND      (SuperstructureConstants.kIntakeGroundAngleRadians,  RollerAction.INTAKE,  ShooterAction.IDLE,    SequencerAction.STOP),
  SPIN_UP_SHOOT      (SuperstructureConstants.kShootAngleRadians,         RollerAction.HOLD,    ShooterAction.SPIN_UP, SequencerAction.STOP),
  SHOOT              (SuperstructureConstants.kShootAngleRadians,         RollerAction.INTAKE,  ShooterAction.SHOOT,   SequencerAction.FEED);

  // ─── Fields ────────────────────────────────────────────────────────────────

  public final double armAngleRadians;
  public final RollerAction rollerAction;
  public final ShooterAction shooterAction;
  public final SequencerAction sequencerAction;

  // ─── Constructor ───────────────────────────────────────────────────────────

  SuperstructureState(
      double armAngleRadians,
      RollerAction rollerAction,
      ShooterAction shooterAction,
      SequencerAction sequencerAction) {
    this.armAngleRadians   = armAngleRadians;
    this.rollerAction      = rollerAction;
    this.shooterAction     = shooterAction;
    this.sequencerAction   = sequencerAction;
  }

  // ─── Action Enums ──────────────────────────────────────────────────────────

  /** Desired action for the intake roller. */
  public enum RollerAction {
    STOP,
    INTAKE,
    HOLD
  }

  /** Desired action for the shooter mechanism. */
  public enum ShooterAction {
    STOP,
    IDLE,
    SPIN_UP,
    SHOOT
  }

  /** Desired action for the sequencer feeder motor. */
  public enum SequencerAction {
    STOP,
    FEED
  }
}
