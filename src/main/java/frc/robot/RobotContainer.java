// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

public class RobotContainer {

  // ─── Controllers ───────────────────────────────────────────────────────────

  private final CommandXboxController m_driver   = new CommandXboxController(0);
  private final CommandXboxController m_operator = new CommandXboxController(1);

  // ─── Subsystems ────────────────────────────────────────────────────────────

  private final Elevator    m_elevator    = new Elevator();
  private final Arm         m_arm         = new Arm();
  private final EndEffector m_endEffector = new EndEffector();

  // ─── Superstructure ────────────────────────────────────────────────────────

  private final Superstructure m_superstructure =
      new Superstructure(m_elevator, m_arm, m_endEffector);

  // ─── Constructor ───────────────────────────────────────────────────────────

  public RobotContainer() {
    configureBindings();
  }

  // ─── Button bindings ───────────────────────────────────────────────────────

  private void configureBindings() {
    // Default state — stow when no button is held
    m_superstructure.setDefaultCommand(
        m_superstructure.holdStateCommand(SuperstructureState.STOW));

    // ── Operator bindings ──────────────────────────────────────────────────
    m_operator.a().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.INTAKE_GROUND));

    m_operator.b().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.INTAKE_SOURCE));

    m_operator.x().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SCORE_LOW));

    m_operator.y().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SCORE_MID));

    m_operator.rightBumper().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SCORE_HIGH));

    m_operator.leftBumper().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.CLIMB));

    // ── Driver bindings ────────────────────────────────────────────────────
    // TODO: add drivetrain bindings here
  }

  // ─── Autonomous ────────────────────────────────────────────────────────────

  public Command getAutonomousCommand() {
    // Example auto sequence — chain state transitions
    return m_superstructure.setStateCommand(SuperstructureState.INTAKE_GROUND)
        .andThen(m_superstructure.setStateCommand(SuperstructureState.SCORE_HIGH))
        .andThen(m_superstructure.setStateCommand(SuperstructureState.STOW))
        .withName("Example Auto");
  }
}
