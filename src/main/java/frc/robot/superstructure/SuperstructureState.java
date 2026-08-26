// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

/**
 * Defines every named state the superstructure can be in.
 *
 * <p>Each state carries the target positions for the elevator (meters) and
 * arm (radians), plus the desired end-effector action.
 *
 * <p>Add more states here as your robot's game requirements grow.
 */
public enum SuperstructureState {

  // ─── State             Elevator (m)   Arm (rad)                Intake action ───────
  STOW               (0.00,           Math.toRadians(  0.0),  EndEffectorAction.STOP),
  INTAKE_GROUND      (0.10,           Math.toRadians(-45.0),  EndEffectorAction.INTAKE),
  INTAKE_SOURCE      (0.60,           Math.toRadians( 30.0),  EndEffectorAction.INTAKE),
  SCORE_LOW          (0.30,           Math.toRadians( 45.0),  EndEffectorAction.OUTTAKE),
  SCORE_MID          (0.65,           Math.toRadians( 60.0),  EndEffectorAction.OUTTAKE),
  SCORE_HIGH         (1.10,           Math.toRadians( 75.0),  EndEffectorAction.OUTTAKE),
  CLIMB              (1.20,           Math.toRadians(  0.0),  EndEffectorAction.STOP);

  // ─── Fields ────────────────────────────────────────────────────────────────

  public final double elevatorHeightMeters;
  public final double armAngleRadians;
  public final EndEffectorAction endEffectorAction;

  // ─── Constructor ───────────────────────────────────────────────────────────

  SuperstructureState(
      double elevatorHeightMeters,
      double armAngleRadians,
      EndEffectorAction endEffectorAction) {
    this.elevatorHeightMeters = elevatorHeightMeters;
    this.armAngleRadians      = armAngleRadians;
    this.endEffectorAction    = endEffectorAction;
  }

  // ─── Inner enum ────────────────────────────────────────────────────────────

  /** What the end effector should be doing in a given state. */
  public enum EndEffectorAction {
    STOP,
    INTAKE,
    OUTTAKE
  }
}
