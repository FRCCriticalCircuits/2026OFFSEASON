// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.SequencerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOKraken;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIOKraken;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.sequencer.Sequencer;
import frc.robot.subsystems.sequencer.SequencerIOKraken;
import frc.robot.subsystems.sequencer.SequencerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIOKraken;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

public class RobotContainer {

  // ─── Controller ────────────────────────────────────────────────────────────

  private final CommandXboxController m_driverController =
      new CommandXboxController(DriverConstants.kDriverControllerPort);

  // ─── Subsystems ────────────────────────────────────────────────────────────

  private final Sequencer m_sequencer;
  private final Arm m_arm;
  private final Roller m_roller;
  private final Shooter m_shooter;
  private final SwerveDrive m_swerveDrive;

  // ─── Superstructure ────────────────────────────────────────────────────────

  private final Superstructure m_superstructure;

  // ─── Constructor ───────────────────────────────────────────────────────────

  public RobotContainer() {
    // Automatically switch between real Kraken hardware on CANivore and WPILib simulation
    if (RobotBase.isReal()) {
      m_sequencer = new Sequencer(new SequencerIOKraken(SequencerConstants.kMotorId));
      m_arm = new Arm(new ArmIOKraken(ArmConstants.kMotorId));
      m_roller = new Roller(new RollerIOKraken(RollerConstants.kMotorId));
      m_shooter =
          new Shooter(
              new ShooterIOKraken(
                  ShooterConstants.kFlywheelLeaderMotorId,
                  ShooterConstants.kFlywheelFollowerMotorId,
                  ShooterConstants.kHoodMotorId));
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
      m_roller = new Roller(new RollerIOSim());
      m_shooter = new Shooter(new ShooterIOSim());
      m_swerveDrive =
          new SwerveDrive(
              new GyroIOSim(),
              new SwerveModuleIOSim(),
              new SwerveModuleIOSim(),
              new SwerveModuleIOSim(),
              new SwerveModuleIOSim());
    }

    m_superstructure = new Superstructure(m_sequencer, m_arm, m_roller, m_shooter);

    configureButtonBindings();
  }

  // ─── Button bindings ───────────────────────────────────────────────────────

  private void configureButtonBindings() {
    // ── 1. Swerve Drive Default Command ─────────────────────────────────────
    m_swerveDrive.setDefaultCommand(
        m_swerveDrive.driveCommand(
            () -> -m_driverController.getLeftY(),  // Forward / backward translation (inverted for field-relative)
            () -> -m_driverController.getLeftX(),  // Left / right translation (inverted for field-relative)
            () -> -m_driverController.getRightX(), // Rotation
            true                                   // Field-relative driving
        ));

    // Reset gyro heading — Start button
    m_driverController.start().onTrue(
        Commands.runOnce(() -> m_swerveDrive.resetHeading(), m_swerveDrive));

    // ── 2. Superstructure Default Command (Stow when no trigger/button held) ─
    m_superstructure.setDefaultCommand(
        m_superstructure.holdStateCommand(SuperstructureState.STOW));

    // ── 3. Single Driver Action Bindings ────────────────────────────────────

    // INTAKE: Left Trigger (Ground Intake) & Right Bumper (Source Intake)
    m_driverController.leftTrigger().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.INTAKE_GROUND));

    m_driverController.rightBumper().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.INTAKE_SOURCE));

    // SHOOT: Right Trigger (Automated Shoot Sequence: Spool Flywheel + Set Hood -> Feed Balls)
    m_driverController.rightTrigger().whileTrue(
        m_superstructure.shootSequenceCommand());

    // OUTTAKE / EJECT: Left Bumper (Purge balls in reverse)
    m_driverController.leftBumper().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.OUTTAKE_EJECT));

    // SCORING PRESETS: A Button (Low Goal), Y Button (High Goal), X Button (Spin Up Pre-spool)
    m_driverController.a().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SCORE_LOW));

    m_driverController.y().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SCORE_HIGH));

    m_driverController.x().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.SPIN_UP_SHOOT));

    m_driverController.b().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.STOW));

    // CLIMB: D-Pad Up
    m_driverController.povUp().whileTrue(
        m_superstructure.holdStateCommand(SuperstructureState.CLIMB));
  }

  // ─── Autonomous ────────────────────────────────────────────────────────────

  public Command getAutonomousCommand() {
    // Example autonomous sequence — Intake -> Shoot -> Stow
    return m_superstructure.setStateCommand(SuperstructureState.INTAKE_GROUND)
        .andThen(Commands.waitSeconds(1.5))
        .andThen(m_superstructure.shootSequenceCommand().withTimeout(2.5))
        .andThen(m_superstructure.setStateCommand(SuperstructureState.STOW))
        .withName("Example Autonomous Command");
  }
}
