// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.sequencer.Sequencer;
import frc.robot.subsystems.sequencer.SequencerIOInputsAutoLogged;
import frc.robot.subsystems.sequencer.SequencerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIOInputsAutoLogged;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.superstructure.Superstructure;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoAim.AutoAimResult;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.LogTable;

/**
 * Unit tests verifying the AdvantageKit logging architecture, auto-logging serialization,
 * defensive cloning, subsystem logging execution, Swerve telemetry, and build constants.
 */
public class AdvantageKitLoggingTest {

  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0);
  }

  @Test
  public void testRobotInheritsLoggedRobot() {
    assertTrue(
        LoggedRobot.class.isAssignableFrom(Robot.class),
        "Robot must extend Littleton Robotics LoggedRobot");
  }

  @Test
  public void testBuildConstantsMetadataAvailable() {
    assertNotNull(BuildConstants.MAVEN_NAME);
    assertEquals("2026OFFSEASON", BuildConstants.MAVEN_NAME);
    assertNotNull(BuildConstants.GIT_SHA);
    assertNotNull(BuildConstants.GIT_BRANCH);
    assertNotNull(BuildConstants.BUILD_DATE);
    assertTrue(BuildConstants.BUILD_UNIX_TIME > 0);
  }

  @Test
  public void testLoggingConstants() {
    assertNotNull(Constants.LoggingConstants.kCTRELogPath);
    assertEquals("/media/sda1/ctre-logs/", Constants.LoggingConstants.kCTRELogPath);
    assertFalse(Constants.LoggingConstants.kCompetitionMode);
    assertFalse(Constants.LoggingConstants.kSysIdSwerve);
  }

  @Test
  public void testArmIOInputsSerializationAndCloning() {
    ArmIOInputsAutoLogged original = new ArmIOInputsAutoLogged();
    original.angleRadians = 1.234;
    original.velocityRadiansPerSecond = 5.678;
    original.appliedVolts = 11.5;
    original.currentAmps = 24.2;
    original.forwardLimitSwitchTripped = true;
    original.reverseLimitSwitchTripped = false;

    LogTable table = new LogTable(0);
    original.toLog(table);

    ArmIOInputsAutoLogged deserialized = new ArmIOInputsAutoLogged();
    deserialized.fromLog(table);

    assertEquals(original.angleRadians, deserialized.angleRadians, 1e-6);
    assertEquals(original.velocityRadiansPerSecond, deserialized.velocityRadiansPerSecond, 1e-6);
    assertEquals(original.appliedVolts, deserialized.appliedVolts, 1e-6);
    assertEquals(original.currentAmps, deserialized.currentAmps, 1e-6);
    assertEquals(original.forwardLimitSwitchTripped, deserialized.forwardLimitSwitchTripped);
    assertEquals(original.reverseLimitSwitchTripped, deserialized.reverseLimitSwitchTripped);

    ArmIOInputsAutoLogged cloned = original.clone();
    assertEquals(original.angleRadians, cloned.angleRadians, 1e-6);
    assertEquals(original.velocityRadiansPerSecond, cloned.velocityRadiansPerSecond, 1e-6);
    assertEquals(original.appliedVolts, cloned.appliedVolts, 1e-6);
    assertEquals(original.currentAmps, cloned.currentAmps, 1e-6);
    assertEquals(original.forwardLimitSwitchTripped, cloned.forwardLimitSwitchTripped);
    assertEquals(original.reverseLimitSwitchTripped, cloned.reverseLimitSwitchTripped);
  }

  @Test
  public void testRollerIOInputsSerializationAndCloning() {
    RollerIOInputsAutoLogged original = new RollerIOInputsAutoLogged();
    original.velocityRotationsPerSecond = 42.0;
    original.appliedVolts = 8.5;
    original.leaderCurrentAmps = 15.2;
    original.followerCurrentAmps = 14.8;
    original.currentAmps = 15.2;
    original.gamePieceDetected = true;

    LogTable table = new LogTable(0);
    original.toLog(table);

    RollerIOInputsAutoLogged deserialized = new RollerIOInputsAutoLogged();
    deserialized.fromLog(table);

    assertEquals(original.velocityRotationsPerSecond, deserialized.velocityRotationsPerSecond, 1e-6);
    assertEquals(original.appliedVolts, deserialized.appliedVolts, 1e-6);
    assertEquals(original.leaderCurrentAmps, deserialized.leaderCurrentAmps, 1e-6);
    assertEquals(original.followerCurrentAmps, deserialized.followerCurrentAmps, 1e-6);
    assertEquals(original.currentAmps, deserialized.currentAmps, 1e-6);
    assertEquals(original.gamePieceDetected, deserialized.gamePieceDetected);

    RollerIOInputsAutoLogged cloned = original.clone();
    assertEquals(original.velocityRotationsPerSecond, cloned.velocityRotationsPerSecond, 1e-6);
    assertEquals(original.appliedVolts, cloned.appliedVolts, 1e-6);
    assertEquals(original.leaderCurrentAmps, cloned.leaderCurrentAmps, 1e-6);
    assertEquals(original.followerCurrentAmps, cloned.followerCurrentAmps, 1e-6);
    assertEquals(original.currentAmps, cloned.currentAmps, 1e-6);
    assertEquals(original.gamePieceDetected, cloned.gamePieceDetected);
  }

  @Test
  public void testSequencerIOInputsSerializationAndCloning() {
    SequencerIOInputsAutoLogged original = new SequencerIOInputsAutoLogged();
    original.velocityRotationsPerSecond = 30.0;
    original.positionRotations = 100.5;
    original.appliedVolts = 10.0;
    original.currentAmps = 18.0;
    original.leaderCurrentAmps = 9.2;
    original.followerCurrentAmps = 8.8;

    LogTable table = new LogTable(0);
    original.toLog(table);

    SequencerIOInputsAutoLogged deserialized = new SequencerIOInputsAutoLogged();
    deserialized.fromLog(table);

    assertEquals(original.velocityRotationsPerSecond, deserialized.velocityRotationsPerSecond, 1e-6);
    assertEquals(original.positionRotations, deserialized.positionRotations, 1e-6);
    assertEquals(original.appliedVolts, deserialized.appliedVolts, 1e-6);
    assertEquals(original.currentAmps, deserialized.currentAmps, 1e-6);
    assertEquals(original.leaderCurrentAmps, deserialized.leaderCurrentAmps, 1e-6);
    assertEquals(original.followerCurrentAmps, deserialized.followerCurrentAmps, 1e-6);

    SequencerIOInputsAutoLogged cloned = original.clone();
    assertEquals(original.velocityRotationsPerSecond, cloned.velocityRotationsPerSecond, 1e-6);
    assertEquals(original.positionRotations, cloned.positionRotations, 1e-6);
    assertEquals(original.appliedVolts, cloned.appliedVolts, 1e-6);
    assertEquals(original.currentAmps, cloned.currentAmps, 1e-6);
    assertEquals(original.leaderCurrentAmps, cloned.leaderCurrentAmps, 1e-6);
    assertEquals(original.followerCurrentAmps, cloned.followerCurrentAmps, 1e-6);
  }

  @Test
  public void testShooterIOInputsSerializationAndCloning() {
    ShooterIOInputsAutoLogged original = new ShooterIOInputsAutoLogged();
    original.flywheelVelocityRotationsPerSecond = 80.0;
    original.flywheelTargetVelocityRotationsPerSecond = 80.0;
    original.flywheelAppliedVolts = 11.2;
    original.flywheelLeaderCurrentAmps = 25.0;
    original.flywheelFollower1CurrentAmps = 24.5;
    original.flywheelFollowerCurrentAmps = 24.5;
    original.flywheelFollower2CurrentAmps = 24.8;
    original.flywheelFollower3CurrentAmps = 25.1;
    original.hoodAngleRadians = 0.52;
    original.hoodTargetAngleRadians = 0.52;
    original.hoodAppliedVolts = 3.2;
    original.hoodCurrentAmps = 4.1;
    original.supportingShooterVelocityRotationsPerSecond = 50.0;
    original.supportingShooterTargetVelocityRotationsPerSecond = 50.0;
    original.supportingShooterAppliedVolts = 7.0;
    original.supportingShooterCurrentAmps = 8.5;

    LogTable table = new LogTable(0);
    original.toLog(table);

    ShooterIOInputsAutoLogged deserialized = new ShooterIOInputsAutoLogged();
    deserialized.fromLog(table);

    assertEquals(original.flywheelVelocityRotationsPerSecond, deserialized.flywheelVelocityRotationsPerSecond, 1e-6);
    assertEquals(original.flywheelTargetVelocityRotationsPerSecond, deserialized.flywheelTargetVelocityRotationsPerSecond, 1e-6);
    assertEquals(original.flywheelAppliedVolts, deserialized.flywheelAppliedVolts, 1e-6);
    assertEquals(original.flywheelLeaderCurrentAmps, deserialized.flywheelLeaderCurrentAmps, 1e-6);
    assertEquals(original.flywheelFollower1CurrentAmps, deserialized.flywheelFollower1CurrentAmps, 1e-6);
    assertEquals(original.flywheelFollowerCurrentAmps, deserialized.flywheelFollowerCurrentAmps, 1e-6);
    assertEquals(original.flywheelFollower2CurrentAmps, deserialized.flywheelFollower2CurrentAmps, 1e-6);
    assertEquals(original.flywheelFollower3CurrentAmps, deserialized.flywheelFollower3CurrentAmps, 1e-6);
    assertEquals(original.hoodAngleRadians, deserialized.hoodAngleRadians, 1e-6);
    assertEquals(original.hoodTargetAngleRadians, deserialized.hoodTargetAngleRadians, 1e-6);
    assertEquals(original.hoodAppliedVolts, deserialized.hoodAppliedVolts, 1e-6);
    assertEquals(original.hoodCurrentAmps, deserialized.hoodCurrentAmps, 1e-6);
    assertEquals(original.supportingShooterVelocityRotationsPerSecond, deserialized.supportingShooterVelocityRotationsPerSecond, 1e-6);
    assertEquals(original.supportingShooterTargetVelocityRotationsPerSecond, deserialized.supportingShooterTargetVelocityRotationsPerSecond, 1e-6);
    assertEquals(original.supportingShooterAppliedVolts, deserialized.supportingShooterAppliedVolts, 1e-6);
    assertEquals(original.supportingShooterCurrentAmps, deserialized.supportingShooterCurrentAmps, 1e-6);

    ShooterIOInputsAutoLogged cloned = original.clone();
    assertEquals(original.flywheelVelocityRotationsPerSecond, cloned.flywheelVelocityRotationsPerSecond, 1e-6);
    assertEquals(original.flywheelAppliedVolts, cloned.flywheelAppliedVolts, 1e-6);
    assertEquals(original.flywheelLeaderCurrentAmps, cloned.flywheelLeaderCurrentAmps, 1e-6);
    assertEquals(original.flywheelFollower1CurrentAmps, cloned.flywheelFollower1CurrentAmps, 1e-6);
    assertEquals(original.hoodAngleRadians, cloned.hoodAngleRadians, 1e-6);
    assertEquals(original.hoodAppliedVolts, cloned.hoodAppliedVolts, 1e-6);
    assertEquals(original.supportingShooterVelocityRotationsPerSecond, cloned.supportingShooterVelocityRotationsPerSecond, 1e-6);
    assertEquals(original.supportingShooterAppliedVolts, cloned.supportingShooterAppliedVolts, 1e-6);
  }

  @Test
  public void testGyroAndSwerveModuleIOInputsSerializationAndCloning() {
    GyroIOInputsAutoLogged gyro = new GyroIOInputsAutoLogged();
    gyro.isConnected = true;
    gyro.yawAngle = Rotation2d.fromDegrees(90.0);
    gyro.yawRateDegreesPerSecond = 15.0;

    LogTable gyroTable = new LogTable(0);
    gyro.toLog(gyroTable);

    GyroIOInputsAutoLogged gyroDeserialized = new GyroIOInputsAutoLogged();
    gyroDeserialized.fromLog(gyroTable);

    assertTrue(gyroDeserialized.isConnected);
    assertEquals(gyro.yawAngle.getDegrees(), gyroDeserialized.yawAngle.getDegrees(), 1e-4);
    assertEquals(gyro.yawRateDegreesPerSecond, gyroDeserialized.yawRateDegreesPerSecond, 1e-6);

    GyroIOInputsAutoLogged gyroCloned = gyro.clone();
    assertTrue(gyroCloned.isConnected);
    assertEquals(gyro.yawAngle.getDegrees(), gyroCloned.yawAngle.getDegrees(), 1e-4);
    assertEquals(gyro.yawRateDegreesPerSecond, gyroCloned.yawRateDegreesPerSecond, 1e-6);

    SwerveModuleIOInputsAutoLogged module = new SwerveModuleIOInputsAutoLogged();
    module.drivePositionMeters = 12.34;
    module.driveVelocityMetersPerSecond = 3.5;
    module.driveAppliedVolts = 9.0;
    module.driveCurrentAmps = 20.0;
    module.steerAngle = Rotation2d.fromDegrees(45.0);
    module.steerVelocityRadiansPerSecond = 1.2;
    module.steerAppliedVolts = 4.5;
    module.steerCurrentAmps = 6.0;

    LogTable moduleTable = new LogTable(0);
    module.toLog(moduleTable);

    SwerveModuleIOInputsAutoLogged moduleDeserialized = new SwerveModuleIOInputsAutoLogged();
    moduleDeserialized.fromLog(moduleTable);

    assertEquals(module.drivePositionMeters, moduleDeserialized.drivePositionMeters, 1e-6);
    assertEquals(module.driveVelocityMetersPerSecond, moduleDeserialized.driveVelocityMetersPerSecond, 1e-6);
    assertEquals(module.driveAppliedVolts, moduleDeserialized.driveAppliedVolts, 1e-6);
    assertEquals(module.driveCurrentAmps, moduleDeserialized.driveCurrentAmps, 1e-6);
    assertEquals(module.steerAngle.getDegrees(), moduleDeserialized.steerAngle.getDegrees(), 1e-4);
    assertEquals(module.steerVelocityRadiansPerSecond, moduleDeserialized.steerVelocityRadiansPerSecond, 1e-6);
    assertEquals(module.steerAppliedVolts, moduleDeserialized.steerAppliedVolts, 1e-6);
    assertEquals(module.steerCurrentAmps, moduleDeserialized.steerCurrentAmps, 1e-6);

    SwerveModuleIOInputsAutoLogged moduleCloned = module.clone();
    assertEquals(module.drivePositionMeters, moduleCloned.drivePositionMeters, 1e-6);
    assertEquals(module.driveVelocityMetersPerSecond, moduleCloned.driveVelocityMetersPerSecond, 1e-6);
    assertEquals(module.driveAppliedVolts, moduleCloned.driveAppliedVolts, 1e-6);
    assertEquals(module.driveCurrentAmps, moduleCloned.driveCurrentAmps, 1e-6);
    assertEquals(module.steerAngle.getDegrees(), moduleCloned.steerAngle.getDegrees(), 1e-4);
    assertEquals(module.steerVelocityRadiansPerSecond, moduleCloned.steerVelocityRadiansPerSecond, 1e-6);
    assertEquals(module.steerAppliedVolts, moduleCloned.steerAppliedVolts, 1e-6);
    assertEquals(module.steerCurrentAmps, moduleCloned.steerCurrentAmps, 1e-6);
  }

  @Test
  public void testSubsystemPeriodicExecutionWithLogging() {
    Arm arm = new Arm(new ArmIOSim());
    Roller roller = new Roller(new RollerIOSim());
    Sequencer sequencer = new Sequencer(new SequencerIOSim());
    Shooter shooter = new Shooter(new ShooterIOSim());
    SwerveDrive swerveDrive =
        new SwerveDrive(
            new GyroIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());
    Superstructure superstructure = new Superstructure(sequencer, arm, roller, shooter);

    // Run multiple periodic cycles ensuring Logger.processInputs and Logger.recordOutput execute cleanly
    for (int i = 0; i < 5; i++) {
      arm.periodic();
      roller.periodic();
      sequencer.periodic();
      shooter.periodic();
      swerveDrive.periodic();
      superstructure.periodic();
    }

    assertNotNull(arm.getAngleRadians());
    assertNotNull(roller.getVelocityRotationsPerSecond());
    assertNotNull(sequencer.getVelocityRotationsPerSecond());
    assertNotNull(shooter.getFlywheelVelocityRotationsPerSecond());
    assertNotNull(swerveDrive.getPose());
  }

  @Test
  public void testSwerveDriveTargetModuleStatesLogging() {
    SwerveDrive swerveDrive =
        new SwerveDrive(
            new GyroIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());

    SwerveModuleState[] initialTargets = swerveDrive.getTargetModuleStates();
    assertNotNull(initialTargets);
    assertEquals(4, initialTargets.length);

    // Command drive speed
    swerveDrive.drive(2.0, 1.0, 0.5, false);
    swerveDrive.periodic();

    SwerveModuleState[] updatedTargets = swerveDrive.getTargetModuleStates();
    assertNotNull(updatedTargets);
    assertEquals(4, updatedTargets.length);
    assertTrue(updatedTargets[0].speedMetersPerSecond > 0.0);
  }

  @Test
  public void testAutoAimVisualizationAndCalculationLogging() {
    Pose2d robotPose = new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(30.0));
    AutoAimResult result = AutoAim.calculate(robotPose);

    assertNotNull(result);
    assertTrue(result.distanceMeters > 0.0);
    assertNotNull(result.targetHeading);
    assertTrue(result.flywheelVelocityRotationsPerSecond > 0.0);
    assertTrue(result.hoodAngleRadians > 0.0);

    Translation2d targetGoal = AutoAim.getTargetGoalLocation();
    assertNotNull(targetGoal);
  }
}
