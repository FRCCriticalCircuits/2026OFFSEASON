// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import frc.robot.Constants.SuperstructureConstants;

/**
 * Defines every named state the superstructure can be in.
 *
 * <p>Each state carries the target positions for the sequencer (meters) and
 * arm (radians), along with the desired actions for the roller and shooter (flywheel + hood).
 */
public enum SuperstructureState {

  // ─── State             Sequencer (m)                                 Arm (rad)                                    RollerAction          ShooterAction ─────
  STOW               (SuperstructureConstants.kStowHeightMeters,          SuperstructureConstants.kStowAngleRadians,          RollerAction.STOP,    ShooterAction.STOP),
  INTAKE_GROUND      (SuperstructureConstants.kIntakeGroundHeightMeters,  SuperstructureConstants.kIntakeGroundAngleRadians,  RollerAction.INTAKE,  ShooterAction.IDLE),
  INTAKE_SOURCE      (SuperstructureConstants.kIntakeSourceHeightMeters,  SuperstructureConstants.kIntakeSourceAngleRadians,  RollerAction.INTAKE,  ShooterAction.IDLE),
  SPIN_UP_SHOOT      (SuperstructureConstants.kShootHeightMeters,         SuperstructureConstants.kShootAngleRadians,         RollerAction.HOLD,    ShooterAction.SPIN_UP_HIGH),
  SHOOT              (SuperstructureConstants.kShootHeightMeters,         SuperstructureConstants.kShootAngleRadians,         RollerAction.INTAKE,  ShooterAction.SHOOT_HIGH),
  SCORE_LOW          (SuperstructureConstants.kScoreLowHeightMeters,      SuperstructureConstants.kScoreLowAngleRadians,      RollerAction.OUTTAKE, ShooterAction.SPIN_UP_LOW),
  SCORE_HIGH         (SuperstructureConstants.kScoreHighHeightMeters,     SuperstructureConstants.kScoreHighAngleRadians,     RollerAction.HOLD,    ShooterAction.SPIN_UP_HIGH),
  OUTTAKE_EJECT      (SuperstructureConstants.kIntakeGroundHeightMeters,  SuperstructureConstants.kIntakeGroundAngleRadians,  RollerAction.OUTTAKE, ShooterAction.STOP),
  CLIMB              (SuperstructureConstants.kClimbHeightMeters,         SuperstructureConstants.kClimbAngleRadians,         RollerAction.STOP,    ShooterAction.STOP);

  // ─── Fields ────────────────────────────────────────────────────────────────

  public final double sequencerHeightMeters;
  public final double armAngleRadians;
  public final RollerAction rollerAction;
  public final ShooterAction shooterAction;

  // ─── Constructor ───────────────────────────────────────────────────────────

  SuperstructureState(
      double sequencerHeightMeters,
      double armAngleRadians,
      RollerAction rollerAction,
      ShooterAction shooterAction) {
    this.sequencerHeightMeters = sequencerHeightMeters;
    this.armAngleRadians       = armAngleRadians;
    this.rollerAction          = rollerAction;
    this.shooterAction         = shooterAction;
  }

  // ─── Action Enums ──────────────────────────────────────────────────────────

  /** Desired action for the intake roller. */
  public enum RollerAction {
    STOP,
    INTAKE,
    OUTTAKE,
    HOLD
  }

  /** Desired action for the shooter mechanism (Flywheel + Hood). */
  public enum ShooterAction {
    STOP,
    IDLE,
    SPIN_UP_LOW,
    SPIN_UP_HIGH,
    SHOOT_LOW,
    SHOOT_HIGH
  }
}
