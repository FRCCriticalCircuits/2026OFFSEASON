// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIOKraken;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.sequencer.Sequencer;
import frc.robot.subsystems.sequencer.SequencerIOSparkMax;
import frc.robot.subsystems.sequencer.SequencerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.GyroIO;
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

  // ─── Autonomous Chooser ────────────────────────────────────────────────────

  private final SendableChooser<Command> m_autoChooser;

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
    // Automatically switch between real hardware and WPILib simulation
    if (RobotBase.isReal()) {
      m_sequencer =
          new Sequencer(
              new SequencerIOSparkMax(
                  SequencerConstants.kLeaderMotorId,
                  SequencerConstants.kFollowerMotorId));
      m_arm = new Arm(new ArmIOSparkMax(ArmConstants.kMotorId));
      m_roller =
          new Roller(
              new RollerIOKraken(
                  RollerConstants.kLeaderMotorId,
                  RollerConstants.kFollowerMotorId));
      m_shooter =
          new Shooter(
              new ShooterIOHardware(
                  ShooterConstants.kFlywheelLeaderMotorId,
                  ShooterConstants.kFlywheelFollower1MotorId,
                  ShooterConstants.kFlywheelFollower2MotorId,
                  ShooterConstants.kFlywheelFollower3MotorId,
                  ShooterConstants.kHoodMotorId,
                  ShooterConstants.kSupportingShooterMotorId));
      m_swerveDrive =
          new SwerveDrive(
              new GyroIO() {},
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

    // Register all superstructure named commands before building the auto chooser
    registerNamedCommands();

    // Build PathPlanner auto chooser and publish to SmartDashboard
    if (AutoBuilder.isConfigured()) {
      m_autoChooser = AutoBuilder.buildAutoChooser("Mobility");
    } else {
      m_autoChooser = new SendableChooser<>();
    }
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    configureButtonBindings();
  }

  // ─── Named Commands Registration ──────────────────────────────────────────

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "Intake",
        m_superstructure.intakeSequenceCommand().withTimeout(2.5));

    NamedCommands.registerCommand(
        "AutoAimShoot",
        m_superstructure.autoAimAndShootCommand(m_swerveDrive, () -> 0.0, () -> 0.0)
            .withTimeout(2.0));

    NamedCommands.registerCommand(
        "Stow",
        m_superstructure.setStateCommand(SuperstructureState.STOW));

    NamedCommands.registerCommand(
        "SpinUp",
        m_superstructure.setStateCommand(SuperstructureState.SPIN_UP_SHOOT).withTimeout(1.0));

    NamedCommands.registerCommand(
        "Eject",
        Commands.startEnd(m_roller::runOuttake, m_roller::stop, m_superstructure).withTimeout(1.0));

    NamedCommands.registerCommand(
        "Shoot",
        m_superstructure.setStateCommand(SuperstructureState.SHOOT).withTimeout(1.5));
  }

  // ─── Button bindings ───────────────────────────────────────────────────────

  private void configureButtonBindings() {
    // ── 1. Swerve Drive Default Command ─────────────────────────────────────
    m_swerveDrive.setDefaultCommand(
        m_swerveDrive.driveCommand(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1),  // Forward / backward translation
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1),  // Left / right translation
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1), // Rotation
            false                                  // Robot-relative driving (no physical Gyro)
        ));

    // Reset gyro heading — Start button
    m_driverController.start().onTrue(
        Commands.runOnce(() -> m_swerveDrive.resetHeading(), m_swerveDrive));

    // ── 2. Superstructure Default Command (Stow when no trigger is held) ────
    m_superstructure.setDefaultCommand(
        m_superstructure.holdStateCommand(SuperstructureState.STOW));

    // ── 3. Single Driver Action Bindings ────────────────────────────────────

    // INTAKE: Sequential Ground Intake (Arm deploys -> waits for angle -> spins roller -> stows on release)
    m_driverController.leftTrigger(DriverConstants.kTriggerThreshold).whileTrue(
        m_superstructure.intakeSequenceCommand());

    m_driverController.rightBumper().debounce(0.05).whileTrue(
        m_superstructure.manual_shoot());

    m_driverController.leftBumper().debounce(0.05).whileTrue(
        m_superstructure.Temp_intakeCommand());
        
    // SHOOT: Dynamic Auto-Aim & Shoot (Heading lock + dynamic flywheel/hood -> auto-feed)
    m_driverController.rightTrigger(DriverConstants.kTriggerThreshold).whileTrue(
        m_superstructure.autoAimAndShootCommand(
            m_swerveDrive,
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1)));
  }

  // ─── Autonomous ────────────────────────────────────────────────────────────

  public Command getAutonomousCommand() {
    if (m_autoChooser != null && m_autoChooser.getSelected() != null) {
      return m_autoChooser.getSelected();
    }
    return Commands.none();
  }
}
