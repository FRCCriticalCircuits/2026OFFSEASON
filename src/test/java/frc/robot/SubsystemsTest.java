// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.sequencer.Sequencer;
import frc.robot.subsystems.sequencer.SequencerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoAim.AutoAimResult;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests verifying subsystem logic, simulation updates, state transitions, and auto-aim math.
 */
public class SubsystemsTest {

  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0);
  }

  @Test
  public void testRobotContainerInstantiation() {
    // Verifies RobotContainer instantiates without exceptions in simulation
    RobotContainer container = new RobotContainer();
    assertNotNull(container);
    assertNotNull(container.getAutonomousCommand());
  }

  @Test
  public void testArmGoalSetting() {
    Arm arm = new Arm(new ArmIOSim());
    arm.setGoal(Math.toRadians(45.0));
    arm.periodic();
    assertNotNull(arm);
  }

  @Test
  public void testSequencerFeedControl() {
    Sequencer sequencer = new Sequencer(new SequencerIOSim());
    sequencer.feed();
    assertEquals(50.0, sequencer.getTargetVelocityRotationsPerSecond());
    sequencer.periodic();
    sequencer.stop();
    assertEquals(0.0, sequencer.getTargetVelocityRotationsPerSecond());
    sequencer.periodic();
    assertNotNull(sequencer);
  }

  @Test
  public void testRollerActions() {
    assertThrows(NullPointerException.class, () -> new Roller(null));

    RollerIOSim simIO = new RollerIOSim();
    simIO.updateInputs(null); // null safety check

    Roller roller = new Roller(simIO);
    roller.runIntake();
    roller.periodic();
    assertEquals(Constants.RollerConstants.kIntakeAppliedVolts, roller.getAppliedVolts(), 1e-4);
    assertTrue(roller.getLeaderCurrentAmps() >= 0.0);
    assertTrue(roller.getFollowerCurrentAmps() >= 0.0);
    assertEquals(roller.getLeaderCurrentAmps(), roller.getFollowerCurrentAmps(), 1e-4);

    roller.runOuttake();
    roller.periodic();
    assertEquals(Constants.RollerConstants.kEjectAppliedVolts, roller.getAppliedVolts(), 1e-4);

    roller.runHold();
    roller.periodic();
    assertEquals(Constants.RollerConstants.kHoldAppliedVolts, roller.getAppliedVolts(), 1e-4);

    roller.setBrakeMode(true);
    roller.setBrakeMode(false);

    // Voltage clamping test (-15V -> -12V, +15V -> +12V)
    roller.setVoltage(15.0);
    roller.periodic();
    assertEquals(12.0, roller.getAppliedVolts(), 1e-4);
    roller.setVoltage(-15.0);
    roller.periodic();
    assertEquals(-12.0, roller.getAppliedVolts(), 1e-4);

    // Non-finite voltage input test (NaN, +Inf, -Inf -> 0.0V)
    roller.setVoltage(Double.NaN);
    roller.periodic();
    assertEquals(0.0, roller.getAppliedVolts(), 1e-4);
    roller.setVoltage(Double.POSITIVE_INFINITY);
    roller.periodic();
    assertEquals(0.0, roller.getAppliedVolts(), 1e-4);
    roller.setVoltage(Double.NEGATIVE_INFINITY);
    roller.periodic();
    assertEquals(0.0, roller.getAppliedVolts(), 1e-4);

    roller.stop();
    roller.periodic();
    assertEquals(0.0, roller.getAppliedVolts(), 1e-4);
    assertEquals(roller.getLeaderCurrentAmps(), roller.getCurrentAmps(), 1e-4);
    assertFalse(roller.isGamePieceDetected());
  }

  @Test
  public void testRollerCommands() {
    Roller roller = new Roller(new RollerIOSim());
    var intakeCmd = roller.intakeCommand();
    assertNotNull(intakeCmd);
    var outtakeCmd = roller.outtakeCommand();
    assertNotNull(outtakeCmd);
    var stopCmd = roller.stopCommand();
    assertNotNull(stopCmd);
  }

  @Test
  public void testRollerDualKrakenSimulationDynamics() {
    RollerIOSim simIO = new RollerIOSim();
    Roller roller = new Roller(simIO);

    // Initial state
    roller.periodic();
    assertEquals(0.0, roller.getVelocityRotationsPerSecond(), 1e-4);
    assertEquals(0.0, roller.getLeaderCurrentAmps(), 1e-4);
    assertEquals(0.0, roller.getFollowerCurrentAmps(), 1e-4);

    // Spin up intake (8V applied) across multiple sim cycles
    roller.runIntake();
    for (int i = 0; i < 25; i++) {
      roller.periodic();
    }
    assertEquals(Constants.RollerConstants.kIntakeAppliedVolts, roller.getAppliedVolts(), 1e-4);
    assertTrue(roller.getVelocityRotationsPerSecond() > 0.0, "Velocity should increase under intake voltage");
    assertTrue(roller.getLeaderCurrentAmps() >= 0.0, "Leader current should be non-negative");
    assertTrue(roller.getFollowerCurrentAmps() >= 0.0, "Follower current should be non-negative");
    assertEquals(roller.getLeaderCurrentAmps(), roller.getFollowerCurrentAmps(), 1e-4, "Dual motors share current in sim");

    // Reverse to eject
    roller.runOuttake();
    for (int i = 0; i < 50; i++) {
      roller.periodic();
    }
    assertEquals(Constants.RollerConstants.kEjectAppliedVolts, roller.getAppliedVolts(), 1e-4);
    assertTrue(roller.getVelocityRotationsPerSecond() < 0.0, "Velocity should reverse under eject voltage");

    // Stop and coast to rest
    roller.stop();
    for (int i = 0; i < 50; i++) {
      roller.periodic();
    }
    assertEquals(0.0, roller.getAppliedVolts(), 1e-4);
  }

  @Test
  public void testRollerCustomMockIOInputs() {
    class MockRollerIO implements frc.robot.subsystems.roller.RollerIO {
      double setVolts = 0.0;
      boolean brakeMode = false;

      @Override
      public void updateInputs(RollerIOInputs inputs) {
        inputs.velocityRotationsPerSecond = 42.0;
        inputs.appliedVolts = setVolts;
        inputs.leaderCurrentAmps = 15.5;
        inputs.followerCurrentAmps = 16.2;
        inputs.currentAmps = inputs.leaderCurrentAmps;
        inputs.gamePieceDetected = true;
      }

      @Override
      public void setVoltage(double appliedVolts) {
        this.setVolts = appliedVolts;
      }

      @Override
      public void setBrakeMode(boolean enableBrakeMode) {
        this.brakeMode = enableBrakeMode;
      }
    }

    MockRollerIO mockIO = new MockRollerIO();
    Roller roller = new Roller(mockIO);
    roller.periodic();

    assertEquals(42.0, roller.getVelocityRotationsPerSecond(), 1e-4);
    assertEquals(0.0, roller.getAppliedVolts(), 1e-4);
    assertEquals(15.5, roller.getLeaderCurrentAmps(), 1e-4);
    assertEquals(16.2, roller.getFollowerCurrentAmps(), 1e-4);
    assertEquals(15.5, roller.getCurrentAmps(), 1e-4);
    assertTrue(roller.isGamePieceDetected());

    roller.runIntake();
    roller.periodic();
    assertEquals(Constants.RollerConstants.kIntakeAppliedVolts, roller.getAppliedVolts(), 1e-4);

    roller.setBrakeMode(true);
    assertTrue(mockIO.brakeMode);
  }

  @Test
  public void testRollerIOKrakenConstructors() {
    // Verifies RollerIOKraken constructors instantiate properly
    var krakenIO1 = new frc.robot.subsystems.roller.RollerIOKraken(
        Constants.RollerConstants.kLeaderMotorId,
        Constants.RollerConstants.kFollowerMotorId);
    assertNotNull(krakenIO1);

    var krakenIO2 = new frc.robot.subsystems.roller.RollerIOKraken(
        Constants.RollerConstants.kLeaderMotorId);
    assertNotNull(krakenIO2);

    krakenIO1.setVoltage(6.0);
    krakenIO1.setVoltage(15.0); // clamped to 12.0
    krakenIO1.setVoltage(-15.0); // clamped to -12.0
    krakenIO1.setVoltage(Double.NaN); // non-finite safely sets 0.0V
    krakenIO1.setVoltage(Double.POSITIVE_INFINITY); // non-finite safely sets 0.0V
    krakenIO1.setBrakeMode(true);
    krakenIO1.setBrakeMode(false);

    krakenIO1.updateInputs(null); // null safety check

    frc.robot.subsystems.roller.RollerIO.RollerIOInputs inputs =
        new frc.robot.subsystems.roller.RollerIO.RollerIOInputs();
    krakenIO1.updateInputs(inputs);
    assertNotNull(inputs);

    // Test default interface methods
    frc.robot.subsystems.roller.RollerIO defaultIO = new frc.robot.subsystems.roller.RollerIO() {};
    defaultIO.updateInputs(inputs);
    defaultIO.updateInputs(null);
    defaultIO.setVoltage(12.0);
    defaultIO.setBrakeMode(true);
  }

  @Test
  public void testShooterFlywheelAndHood() {
    Shooter shooter = new Shooter(new ShooterIOSim());
    shooter.prepareShot(70.0, Math.toRadians(35.0));
    assertEquals(70.0, shooter.getTargetFlywheelVelocityRotationsPerSecond());
    assertEquals(Math.toRadians(35.0), shooter.getTargetHoodAngleRadians());
    assertEquals(
        Constants.ShooterConstants.kSupportingShooterTargetVelocityRotationsPerSecond,
        shooter.getTargetSupportingShooterVelocityRotationsPerSecond());
    shooter.periodic();
    shooter.stop();
    assertEquals(0.0, shooter.getTargetFlywheelVelocityRotationsPerSecond());
    assertEquals(0.0, shooter.getTargetSupportingShooterVelocityRotationsPerSecond());
  }

  @Test
  public void testShooterReadyToShoot() {
    Shooter shooter = new Shooter(new ShooterIOSim());
    assertFalse(shooter.isReadyToShoot());

    shooter.setFlywheelVelocity(0.0);
    shooter.setHoodAngle(0.0);
    shooter.stopSupportingShooter();
    shooter.periodic();
    // At zero target speed, atTargetFlywheelSpeed is false
    assertFalse(shooter.isReadyToShoot());
  }

  @Test
  public void testSequencerDualMotorCurrents() {
    Sequencer sequencer = new Sequencer(new SequencerIOSim());
    sequencer.feed();
    sequencer.periodic();
    assertEquals(sequencer.getLeaderCurrentAmps(), sequencer.getFollowerCurrentAmps(), 1e-4);
    assertEquals(
        sequencer.getLeaderCurrentAmps() + sequencer.getFollowerCurrentAmps(),
        sequencer.getCurrentAmps(),
        1e-4);
  }

  @Test
  public void testSuperstructureStateTransitions() {
    Arm arm = new Arm(new ArmIOSim());
    Sequencer sequencer = new Sequencer(new SequencerIOSim());
    Roller roller = new Roller(new RollerIOSim());
    Shooter shooter = new Shooter(new ShooterIOSim());

    Superstructure superstructure = new Superstructure(sequencer, arm, roller, shooter);

    // Initial state
    assertEquals(SuperstructureState.STOW, superstructure.getCurrentState());

    // Sequential intake command
    assertNotNull(superstructure.intakeSequenceCommand());
  }

  @Test
  public void testAutoAimDoesNotCommandArm() {
    Arm arm = new Arm(new ArmIOSim());
    Sequencer sequencer = new Sequencer(new SequencerIOSim());
    Roller roller = new Roller(new RollerIOSim());
    Shooter shooter = new Shooter(new ShooterIOSim());
    SwerveDrive swerve =
        new SwerveDrive(
            new GyroIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());

    Superstructure superstructure = new Superstructure(sequencer, arm, roller, shooter);

    // Arm starts at stow (0 rad)
    arm.setGoal(0.0);
    double initialArmGoal = arm.getGoalAngleRadians();

    var autoAimCmd = superstructure.autoAimAndShootCommand(swerve, () -> 0.0, () -> 0.0);
    assertNotNull(autoAimCmd);

    // Initialize and execute one step of autoaim
    autoAimCmd.initialize();
    autoAimCmd.execute();

    // Verify arm goal was NOT altered by autoaim
    assertEquals(initialArmGoal, arm.getGoalAngleRadians(), 1e-4, "Arm goal should remain untouched during autoaim");

    // End/interrupt autoaim
    autoAimCmd.end(false);
    assertEquals(initialArmGoal, arm.getGoalAngleRadians(), 1e-4, "Arm goal should remain untouched when autoaim ends");
  }

  @Test
  public void testAutoAimCalculations() {
    Pose2d robotPose = new Pose2d(3.0, 5.55, new Rotation2d());
    AutoAimResult result = AutoAim.calculate(robotPose);

    assertNotNull(result);
    assertTrue(result.distanceMeters > 0.0);
    assertNotNull(result.targetHeading);
    assertTrue(result.flywheelVelocityRotationsPerSecond > 0.0);
    assertTrue(result.hoodAngleRadians > 0.0);
  }

  @Test
  public void testSwerveDriveKinematics() {
    SwerveDrive swerve =
        new SwerveDrive(
            new GyroIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());

    swerve.drive(1.0, 0.0, 0.0, true);
    swerve.periodic();
    assertNotNull(swerve.getPose());
    assertEquals(0.0, swerve.getHeading().getDegrees(), 1e-3);
  }

  @Test
  public void testSwerveAutoBuilderIntegration() {
    SwerveDrive swerve =
        new SwerveDrive(
            new GyroIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());

    // Kinematics accessor
    assertNotNull(swerve.getKinematics());

    // Robot relative speeds accessor
    edu.wpi.first.math.kinematics.ChassisSpeeds currentSpeeds = swerve.getRobotRelativeSpeeds();
    assertNotNull(currentSpeeds);

    // Robot relative driving
    swerve.driveRobotRelative(new edu.wpi.first.math.kinematics.ChassisSpeeds(2.0, 1.0, 0.5));
    swerve.periodic();

    // AutoBuilder configured
    assertTrue(com.pathplanner.lib.auto.AutoBuilder.isConfigured());

    // Odometry reset
    swerve.resetOdometry(new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(90)));
    assertEquals(2.0, swerve.getPose().getX(), 1e-3);
    assertEquals(4.0, swerve.getPose().getY(), 1e-3);
  }

  @Test
  public void testShooter6MotorsSim() {
    ShooterIOSim simIO = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

    simIO.setFlywheelVelocity(70.0);
    simIO.setHoodAngle(Math.toRadians(45.0));
    simIO.setSupportingShooterVelocity(60.0);
    simIO.updateInputs(inputs);

    assertEquals(70.0, inputs.flywheelTargetVelocityRotationsPerSecond, 1e-4);
    assertEquals(Math.toRadians(45.0), inputs.hoodTargetAngleRadians, 1e-4);
    assertEquals(60.0, inputs.supportingShooterTargetVelocityRotationsPerSecond, 1e-4);
    assertTrue(inputs.flywheelFollower3CurrentAmps >= 0.0);
    assertEquals(inputs.flywheelLeaderCurrentAmps, inputs.flywheelFollower3CurrentAmps, 1e-4);
    assertTrue(inputs.supportingShooterCurrentAmps >= 0.0);
  }

  @Test
  public void testShooterIOHardwareConstructors() {
    // 6-motor constructor (4 flywheels + 1 hood + 1 supporting shooter)
    var hw6 = new frc.robot.subsystems.shooter.ShooterIOHardware(
        Constants.ShooterConstants.kFlywheelLeaderMotorId,
        Constants.ShooterConstants.kFlywheelFollower1MotorId,
        Constants.ShooterConstants.kFlywheelFollower2MotorId,
        Constants.ShooterConstants.kFlywheelFollower3MotorId,
        Constants.ShooterConstants.kHoodMotorId,
        Constants.ShooterConstants.kSupportingShooterMotorId);
    assertNotNull(hw6);
    hw6.close();

    // Default constructor
    var hwDef = new frc.robot.subsystems.shooter.ShooterIOHardware();
    assertNotNull(hwDef);

    hwDef.setFlywheelVoltage(6.0);
    hwDef.setHoodVoltage(6.0);
    hwDef.setSupportingShooterVoltage(6.0);
    hwDef.stopFlywheel();
    hwDef.stopHood();
    hwDef.stopSupportingShooter();
    hwDef.resetHoodEncoder();

    hwDef.updateInputs(null);
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    hwDef.updateInputs(inputs);
    assertNotNull(inputs);
    hwDef.close();
  }

  @Test
  public void testArmAndSequencerSparkMaxIO() {
    var armSparkMax = new frc.robot.subsystems.arm.ArmIOSparkMax(60);
    assertNotNull(armSparkMax);
    armSparkMax.setVoltage(6.0);
    armSparkMax.setBrakeMode(true);
    armSparkMax.resetEncoder();
    armSparkMax.updateInputs(null);
    armSparkMax.close();

    var seqSparkMax = new frc.robot.subsystems.sequencer.SequencerIOSparkMax(61, 62);
    assertNotNull(seqSparkMax);
    seqSparkMax.setVoltage(6.0);
    seqSparkMax.setBrakeMode(true);
    seqSparkMax.stop();
    seqSparkMax.updateInputs(null);
    seqSparkMax.close();
  }

  @Test
  public void testSwerveModuleKrakenX44SteerSim() {
    SwerveModuleIOSim moduleSim = new SwerveModuleIOSim();
    frc.robot.subsystems.swerve.SwerveModuleIO.SwerveModuleIOInputs inputs =
        new frc.robot.subsystems.swerve.SwerveModuleIO.SwerveModuleIOInputs();
    moduleSim.setSteerAngle(Rotation2d.fromDegrees(45.0));
    moduleSim.setDriveVoltage(6.0);
    moduleSim.updateInputs(inputs);
    assertTrue(inputs.steerCurrentAmps >= 0.0);
    assertTrue(inputs.driveCurrentAmps >= 0.0);
  }
}
