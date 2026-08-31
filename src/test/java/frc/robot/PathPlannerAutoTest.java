// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class PathPlannerAutoTest {

  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0);
    CommandScheduler.getInstance().cancelAll();
  }

  @AfterEach
  public void tearDown() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Test
  public void testAutoBuilderConfigured() {
    RobotContainer container = new RobotContainer();
    assertTrue(AutoBuilder.isConfigured());
  }

  @Test
  public void testRobotConfigFromGUISettings() throws Exception {
    RobotConfig config = RobotConfig.fromGUISettings();
    assertNotNull(config);
    assertEquals(45.0, config.massKG, 1e-3);
    assertEquals(4.0, config.MOI, 1e-3);
    assertEquals(0.0508, config.moduleConfig.wheelRadiusMeters, 1e-4);
    assertEquals(5.12, config.moduleConfig.maxDriveVelocityMPS, 1e-3);
  }

  @Test
  public void testNamedCommandsRegistration() {
    RobotContainer container = new RobotContainer();

    String[] requiredCommands = {
      "Intake", "AutoAimShoot", "Stow", "SpinUp", "Eject", "Shoot"
    };

    for (String commandName : requiredCommands) {
      assertTrue(
          NamedCommands.hasCommand(commandName),
          "Expected NamedCommand '" + commandName + "' to be registered");
      Command cmd = NamedCommands.getCommand(commandName);
      assertNotNull(cmd, "Command '" + commandName + "' should not be null");
    }
  }

  @Test
  public void testDeployPathsParsing() {
    String[] pathNames = {
      "TaxiPath",
      "HubToPiece1",
      "Piece1ToHub",
      "HubToPiece2",
      "Piece2ToHub",
      "CenterlinePiece1",
      "Piece1ToHubScore",
      "CenterlinePiece2",
      "Piece2ToHubScore",
      "CenterlinePiece3"
    };

    for (String pathName : pathNames) {
      assertDoesNotThrow(
          () -> {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            assertNotNull(path, "Path '" + pathName + "' should load successfully");
            assertTrue(
                path.getAllPathPoints().size() > 0 || path.getWaypoints().size() > 0,
                "Path '" + pathName + "' should have valid waypoints");
          },
          "Failed loading path: " + pathName);
    }
  }

  @Test
  public void testDeployAutosParsing() {
    RobotContainer container = new RobotContainer();

    String[] autoNames = {
      "Mobility",
      "3PieceHubScore",
      "4PieceCenterline"
    };

    for (String autoName : autoNames) {
      assertDoesNotThrow(
          () -> {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            assertNotNull(auto, "Auto '" + autoName + "' should load successfully");
          },
          "Failed loading auto routine: " + autoName);
    }
  }

  @Test
  public void testAutoChooserPopulated() {
    RobotContainer container = new RobotContainer();
    Command autoCommand = container.getAutonomousCommand();
    assertNotNull(autoCommand, "getAutonomousCommand() should return a non-null command");
  }

  @Test
  public void testMockTrajectorySimulationExecution() {
    edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
    edu.wpi.first.wpilibj.simulation.DriverStationSim.setAutonomous(true);
    edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();

    RobotContainer container = new RobotContainer();
    Command autoCommand = container.getAutonomousCommand();
    assertNotNull(autoCommand);

    // Schedule autonomous command
    autoCommand.schedule();
    assertTrue(CommandScheduler.getInstance().isScheduled(autoCommand));

    // Run scheduler for 20 periodic iterations (0.4s simulation time)
    for (int cycle = 0; cycle < 20; cycle++) {
      assertDoesNotThrow(() -> CommandScheduler.getInstance().run());
    }

    // Cancel command cleanly
    autoCommand.cancel();
    assertFalse(CommandScheduler.getInstance().isScheduled(autoCommand));

    edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(false);
    edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
  }
}
