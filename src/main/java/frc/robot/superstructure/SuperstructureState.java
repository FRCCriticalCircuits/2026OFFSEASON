// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import frc.robot.Constants.SuperstructureConstants;

/**
 * Defines every named state the superstructure can be in.
 *
 * <p>Each state carries the target positions for the sequencer (meters) and
 * arm (radians).
 *
 * <p>Add more states here as your robot's game requirements grow.
 */
public enum SuperstructureState {

  // ─── State             Sequencer (meters)                                 Arm (radians) ───────
  STOW               (SuperstructureConstants.kStowHeightMeters,          SuperstructureConstants.kStowAngleRadians),
  INTAKE_GROUND      (SuperstructureConstants.kIntakeGroundHeightMeters,  SuperstructureConstants.kIntakeGroundAngleRadians),
  INTAKE_SOURCE      (SuperstructureConstants.kIntakeSourceHeightMeters,  SuperstructureConstants.kIntakeSourceAngleRadians),
  SCORE_LOW          (SuperstructureConstants.kScoreLowHeightMeters,      SuperstructureConstants.kScoreLowAngleRadians),
  SCORE_MID          (SuperstructureConstants.kScoreMidHeightMeters,      SuperstructureConstants.kScoreMidAngleRadians),
  SCORE_HIGH         (SuperstructureConstants.kScoreHighHeightMeters,     SuperstructureConstants.kScoreHighAngleRadians),
  CLIMB              (SuperstructureConstants.kClimbHeightMeters,         SuperstructureConstants.kClimbAngleRadians);

  // ─── Fields ────────────────────────────────────────────────────────────────

  public final double sequencerHeightMeters;
  public final double armAngleRadians;

  // ─── Constructor ───────────────────────────────────────────────────────────

  SuperstructureState(double sequencerHeightMeters, double armAngleRadians) {
    this.sequencerHeightMeters = sequencerHeightMeters;
    this.armAngleRadians      = armAngleRadians;
  }
}
