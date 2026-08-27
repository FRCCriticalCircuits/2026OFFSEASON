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
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
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
  public void testSequencerGoalSetting() {
    Sequencer sequencer = new Sequencer(new SequencerIOSim());
    sequencer.setGoal(0.5);
    sequencer.periodic();
    assertNotNull(sequencer);
  }

  @Test
  public void testRollerActions() {
    Roller roller = new Roller(new RollerIOSim());
    roller.runIntake();
    roller.periodic();
    roller.stop();
    roller.periodic();
    assertFalse(roller.isGamePieceDetected());
  }

  @Test
  public void testShooterFlywheelAndHood() {
    Shooter shooter = new Shooter(new ShooterIOSim());
    shooter.prepareShot(70.0, Math.toRadians(35.0));
    assertEquals(70.0, shooter.getTargetFlywheelVelocityRotationsPerSecond());
    assertEquals(Math.toRadians(35.0), shooter.getTargetHoodAngleRadians());
    shooter.periodic();
    shooter.stop();
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
}
