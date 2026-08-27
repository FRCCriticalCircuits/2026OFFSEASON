// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SequencerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOKraken;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.sequencer.Sequencer;
import frc.robot.subsystems.sequencer.SequencerIOKraken;
import frc.robot.subsystems.sequencer.SequencerIOSim;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIOKraken;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

public class RobotContainer {

  // ─── Controllers ───────────────────────────────────────────────────────────

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // ─── Subsystems ────────────────────────────────────────────────────────────

  private final Sequencer m_sequencer;
  private final Arm m_arm;
  private final SwerveDrive m_swerveDrive;

  // ─── Superstructure ────────────────────────────────────────────────────────

  private final Superstructure m_superstructure;

  // ─── Constructor ───────────────────────────────────────────────────────────

  public RobotContainer() {
    // Automatically switch between real Kraken hardware on CANivore and WPILib simulation
    if (RobotBase.isReal()) {
      m_sequencer = new Sequencer(new SequencerIOKraken(SequencerConstants.kMotorId));
      m_arm = new Arm(new ArmIOKraken(ArmConstants.kMotorId));
      m_swerveDrive =
          new SwerveDrive(
              new GyroIOPigeon2(SwerveConstants.kPigeon2CanId),
              new SwerveModuleIOKraken(
                  SwerveConstants.kFrontLeftDriveMotorId,
                  SwerveConstants.kFrontLeftSteerMotorId,
                  SwerveConstants.kFrontLeftCANcoderId,
                  SwerveConstants.kFrontLeftCANcoderOffsetRotations,
                  SwerveConstants.kFrontLeftDriveInverted),
              new SwerveModuleIOKraken(
                  SwerveConstants.kFrontRightDriveMotorId,
                  SwerveConstants.kFrontRightSteerMotorId,
                  SwerveConstants.kFrontRightCANcoderId,
                  SwerveConstants.kFrontRightCANcoderOffsetRotations,
                  SwerveConstants.kFrontRightDriveInverted),
              new SwerveModuleIOKraken(
                  SwerveConstants.kBackLeftDriveMotorId,
                  SwerveConstants.kBackLeftSteerMotorId,
                  SwerveConstants.kBackLeftCANcoderId,
                  SwerveConstants.kBackLeftCANcoderOffsetRotations,
                  SwerveConstants.kBackLeftDriveInverted),
              new SwerveModuleIOKraken(
                  SwerveConstants.kBackRightDriveMotorId,
                  SwerveConstants.kBackRightSteerMotorId,
                  SwerveConstants.kBackRightCANcoderId,
                  SwerveConstants.kBackRightCANcoderOffsetRotations,
                  SwerveConstants.kBackRightDriveInverted));
    } else {
      m_sequencer = new Sequencer(new SequencerIOSim());
      m_arm = new Arm(new ArmIOSim());
      m_swerveDrive =
          new SwerveDrive(
              new GyroIOSim(),
              new SwerveModuleIOSim(),
              new SwerveModuleIOSim(),
              new SwerveModuleIOSim(),
              new SwerveModuleIOSim());
    }

    m_superstructure = new Superstructure(m_sequencer, m_arm);

    configureButtonBindings();
  }

  // ─── Button bindings ───────────────────────────────────────────────────────

  private void configureButtonBindings() {
    // ── Swerve drive default command ────────────────────────────────────────
    m_swerveDrive.setDefaultCommand(
        m_swerveDrive.driveCommand(
            () -> -m_driverController.getLeftY(),  // Forward / backward translation (inverted for field-relative)
            () -> -m_driverController.getLeftX(),  // Left / right translation (inverted for field-relative)
            () -> -m_driverController.getRightX(), // Rotation
            true                                   // Field-relative driving
        ));

    // Reset gyro heading — driver Start button
    m_driverController.start().onTrue(
        Commands.runOnce(() -> m_swerveDrive.resetHeading(), m_swerveDrive));

    // ── Superstructure default — stow when no button is held ────────────────
    m_superstructure.setDefaultCommand(
        m_superstructure.holdStateCommand(SuperstructureState.STOW));

    // ── Operator bindings ──────────────────────────────────────────────────
    m_operatorController.a().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.INTAKE_GROUND));

    m_operatorController.b().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.INTAKE_SOURCE));

    m_operatorController.x().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SCORE_LOW));

    m_operatorController.y().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SCORE_MID));

    m_operatorController.rightBumper().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SCORE_HIGH));

    m_operatorController.leftBumper().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.CLIMB));
  }

  // ─── Autonomous ────────────────────────────────────────────────────────────

  public Command getAutonomousCommand() {
    // Example autonomous sequence — chain state transitions
    return m_superstructure.setStateCommand(SuperstructureState.INTAKE_GROUND)
        .andThen(m_superstructure.setStateCommand(SuperstructureState.SCORE_HIGH))
        .andThen(m_superstructure.setStateCommand(SuperstructureState.STOW))
        .withName("Example Autonomous Command");
  }
}
