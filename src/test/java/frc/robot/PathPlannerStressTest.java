// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class PathPlannerStressTest {

  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().cancelAll();
  }

  @AfterEach
  public void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  @DisplayName("Stress Test: NamedCommands execution, periodic progression, and timeout termination")
  public void testNamedCommandsLifecycleAndTimeoutTermination() {
    RobotContainer container = new RobotContainer();

    String[] namedCommands = {"Intake", "AutoAimShoot", "Stow", "SpinUp", "Eject", "Shoot"};

    for (String name : namedCommands) {
      assertTrue(NamedCommands.hasCommand(name), "Missing named command: " + name);
      Command cmd = NamedCommands.getCommand(name);
      assertNotNull(cmd, "Command was null: " + name);

      // Schedule command
      CommandScheduler.getInstance().schedule(cmd);
      assertTrue(
          CommandScheduler.getInstance().isScheduled(cmd),
          "Command should be scheduled: " + name);

      // Run scheduler until timeout occurs (up to 3.5 seconds / 175 cycles)
      int maxCycles = 200; // 4 seconds at 50Hz
      int cyclesRan = 0;
      while (CommandScheduler.getInstance().isScheduled(cmd) && cyclesRan < maxCycles) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.020);
        CommandScheduler.getInstance().run();
        cyclesRan++;
      }

      // Assert that the command completed within maxCycles (no infinite hang)
      assertFalse(
          CommandScheduler.getInstance().isScheduled(cmd),
          "Named command '" + name + "' failed to terminate within timeout!");
    }
  }

  @Test
  @DisplayName("Stress Test: NamedCommands early interruption and cleanup")
  public void testNamedCommandsEarlyInterruption() {
    RobotContainer container = new RobotContainer();

    String[] interruptibleCommands = {"Intake", "AutoAimShoot", "SpinUp", "Eject", "Shoot"};

    for (String name : interruptibleCommands) {
      Command cmd = NamedCommands.getCommand(name);
      assertNotNull(cmd);

      CommandScheduler.getInstance().schedule(cmd);
      assertTrue(CommandScheduler.getInstance().isScheduled(cmd));

      // Run for 5 cycles
      for (int i = 0; i < 5; i++) {
        CommandScheduler.getInstance().run();
      }

      // Interrupt by cancelling
      CommandScheduler.getInstance().cancel(cmd);
      assertFalse(CommandScheduler.getInstance().isScheduled(cmd));

      // Run 1 cycle after cancellation
      assertDoesNotThrow(() -> CommandScheduler.getInstance().run());
    }
  }

  @Test
  @DisplayName("Stress Test: Extreme and boundary chassis speeds on SwerveDrive")
  public void testExtremeChassisSpeedsDesaturation() {
    SwerveDrive swerve =
        new SwerveDrive(
            new GyroIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());

    // Normal speeds
    assertDoesNotThrow(() -> swerve.driveRobotRelative(new ChassisSpeeds(1.0, 1.0, 1.0)));
    swerve.periodic();

    // Extreme positive speeds (100 m/s, 50 rad/s)
    assertDoesNotThrow(() -> swerve.driveRobotRelative(new ChassisSpeeds(100.0, 100.0, 50.0)));
    swerve.periodic();

    // Extreme negative speeds (-100 m/s, -50 rad/s)
    assertDoesNotThrow(() -> swerve.driveRobotRelative(new ChassisSpeeds(-100.0, -100.0, -50.0)));
    swerve.periodic();

    // Zero speeds
    assertDoesNotThrow(() -> swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)));
    swerve.periodic();

    // ChassisSpeeds roundtrip check
    ChassisSpeeds speeds = swerve.getRobotRelativeSpeeds();
    assertNotNull(speeds);
    assertFalse(Double.isNaN(speeds.vxMetersPerSecond));
    assertFalse(Double.isNaN(speeds.vyMetersPerSecond));
    assertFalse(Double.isNaN(speeds.omegaRadiansPerSecond));
  }

  @ParameterizedTest
  @ValueSource(doubles = {-10.0, 0.0, 1.5, 8.27, 16.54})
  @DisplayName("Stress Test: Odometry resets across field boundaries")
  public void testOdometryResetsAcrossField(double xCoord) {
    SwerveDrive swerve =
        new SwerveDrive(
            new GyroIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());

    Pose2d targetPose = new Pose2d(xCoord, 4.0, Rotation2d.fromDegrees(45.0));
    swerve.resetOdometry(targetPose);
    swerve.periodic();

    assertEquals(xCoord, swerve.getPose().getX(), 1e-3);
    assertEquals(4.0, swerve.getPose().getY(), 1e-3);
  }

  @Test
  @DisplayName("Stress Test: Dynamic Alliance Switching Simulation")
  public void testAllianceFlippingHandling() {
    RobotContainer container = new RobotContainer();

    // Test with Blue alliance
    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    assertTrue(AutoBuilder.isConfigured());

    // Test with Red alliance
    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    assertTrue(AutoBuilder.isConfigured());
  }

  @Test
  @DisplayName("Stress Test: Full auto routine simulation execution under Autonomous mode")
  public void testFullAutoRoutinesExecutionUnderAutonomous() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();

    RobotContainer container = new RobotContainer();

    String[] autoRoutines = {"Mobility", "3PieceHubScore", "4PieceCenterline"};

    for (String autoName : autoRoutines) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
      assertNotNull(auto, "Auto should instantiate: " + autoName);

      CommandScheduler.getInstance().schedule(auto);
      assertTrue(CommandScheduler.getInstance().isScheduled(auto));

      // Simulate 50 periodic iterations (1.0 second of autonomous run)
      for (int cycle = 0; cycle < 50; cycle++) {
        assertDoesNotThrow(
            () -> CommandScheduler.getInstance().run(),
            "Scheduler threw exception during auto routine " + autoName);
      }

      // Cancel clean
      CommandScheduler.getInstance().cancel(auto);
      assertFalse(CommandScheduler.getInstance().isScheduled(auto));
    }

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  @DisplayName("Stress Test: Rapid sequential NamedCommand chaining")
  public void testRapidSequentialNamedCommandChaining() {
    RobotContainer container = new RobotContainer();

    String[] sequence = {"Intake", "SpinUp", "Shoot", "Stow", "Eject", "Stow"};

    for (String name : sequence) {
      Command cmd = NamedCommands.getCommand(name);
      assertNotNull(cmd);
      CommandScheduler.getInstance().schedule(cmd);
      assertTrue(CommandScheduler.getInstance().isScheduled(cmd));

      // Step 5 cycles
      for (int i = 0; i < 5; i++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.020);
        CommandScheduler.getInstance().run();
      }

      // Next command replaces previous
    }

    CommandScheduler.getInstance().cancelAll();
  }

  @ParameterizedTest
  @ValueSource(doubles = {0.5, 1.5, 4.0, 8.27, 12.0, 16.54})
  @DisplayName("Stress Test: AutoAim calculation stability across field coordinates")
  public void testAutoAimStabilityAcrossField(double xCoord) {
    Pose2d pose = new Pose2d(xCoord, 4.0, Rotation2d.fromDegrees(30.0));
    frc.robot.util.AutoAim.AutoAimResult result = frc.robot.util.AutoAim.calculate(pose);

    assertNotNull(result);
    assertFalse(Double.isNaN(result.distanceMeters), "Distance should not be NaN");
    assertFalse(Double.isNaN(result.flywheelVelocityRotationsPerSecond), "Flywheel speed should not be NaN");
    assertFalse(Double.isNaN(result.hoodAngleRadians), "Hood angle should not be NaN");
    assertNotNull(result.targetHeading);
    assertTrue(result.distanceMeters >= 0.0);
    assertTrue(result.flywheelVelocityRotationsPerSecond >= 0.0);
    assertTrue(result.hoodAngleRadians >= 0.0);
  }

  @Test
  @DisplayName("Stress Test: Auto routine cancel and reschedule cycling")
  public void testAutoRoutineCancelAndRescheduleCycling() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();

    RobotContainer container = new RobotContainer();

    String[] autoRoutines = {"Mobility", "3PieceHubScore", "4PieceCenterline", "Mobility"};

    for (String autoName : autoRoutines) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
      CommandScheduler.getInstance().schedule(auto);
      assertTrue(CommandScheduler.getInstance().isScheduled(auto));

      for (int cycle = 0; cycle < 15; cycle++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.020);
        CommandScheduler.getInstance().run();
      }

      CommandScheduler.getInstance().cancel(auto);
      assertFalse(CommandScheduler.getInstance().isScheduled(auto));
    }

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  @DisplayName("Stress Test: Subsystem requirement conflict resolution")
  public void testSubsystemRequirementConflicts() {
    RobotContainer container = new RobotContainer();

    Command intake = NamedCommands.getCommand("Intake");
    Command spinUp = NamedCommands.getCommand("SpinUp");

    // Both require Superstructure. Scheduling spinUp should interrupt intake.
    CommandScheduler.getInstance().schedule(intake);
    assertTrue(CommandScheduler.getInstance().isScheduled(intake));

    CommandScheduler.getInstance().schedule(spinUp);
    assertTrue(CommandScheduler.getInstance().isScheduled(spinUp));
    assertFalse(
        CommandScheduler.getInstance().isScheduled(intake),
        "Intake should have been interrupted by SpinUp");

    // Clean cancellation
    CommandScheduler.getInstance().cancel(spinUp);
    assertFalse(CommandScheduler.getInstance().isScheduled(spinUp));
  }
}
