// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.subsystems.swerve.SwerveDrive;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    if (Robot.isReal()) {
      java.io.File ctreLogDir = new java.io.File(Constants.LoggingConstants.kCTRELogPath);
      if (!ctreLogDir.exists()) {
        ctreLogDir.mkdirs();
      }
      if (ctreLogDir.exists()) {
        SignalLogger.setPath(Constants.LoggingConstants.kCTRELogPath);
      }
    }

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GIT Status",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers
    Logger.addDataReceiver(new WPILOGWriter());
    if (!Robot.isReal() || !Constants.LoggingConstants.kCompetitionMode) {
      Logger.addDataReceiver(new NT4Publisher());
    }

    if (Constants.LoggingConstants.kSysIdSwerve) {
      SignalLogger.start();
    }
    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SwerveDrive drive = m_robotContainer.getSwerveDrive();
    double omegaRps = Units.degreesToRotations(drive.getTurnRateDegreesPerSecond());
    var llmeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    if (llmeasurement != null && llmeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
      m_robotContainer.getSwerveDrive().resetOdometry(llmeasurement.pose);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

