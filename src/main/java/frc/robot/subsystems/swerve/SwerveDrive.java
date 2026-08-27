// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import java.util.function.DoubleSupplier;

public class SwerveDrive extends SubsystemBase {
  private final GyroIO m_gyroIO;
  private final GyroIO.GyroIOInputs m_gyroInputs = new GyroIO.GyroIOInputs();

  private final SwerveModuleIO[] m_moduleIOs;
  private final SwerveModuleIO.SwerveModuleIOInputs[] m_moduleInputs;

  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;

  public SwerveDrive(
      GyroIO gyroIO,
      SwerveModuleIO frontLeftModuleIO,
      SwerveModuleIO frontRightModuleIO,
      SwerveModuleIO backLeftModuleIO,
      SwerveModuleIO backRightModuleIO) {
    m_gyroIO = gyroIO;
    m_moduleIOs =
        new SwerveModuleIO[] {
          frontLeftModuleIO, frontRightModuleIO, backLeftModuleIO, backRightModuleIO
        };
    m_moduleInputs =
        new SwerveModuleIO.SwerveModuleIOInputs[] {
          new SwerveModuleIO.SwerveModuleIOInputs(),
          new SwerveModuleIO.SwerveModuleIOInputs(),
          new SwerveModuleIO.SwerveModuleIOInputs(),
          new SwerveModuleIO.SwerveModuleIOInputs()
        };

    double halfWheelbaseMeters = SwerveConstants.kWheelbaseMeters / 2.0;
    double halfTrackWidthMeters = SwerveConstants.kTrackWidthMeters / 2.0;

    m_kinematics =
        new SwerveDriveKinematics(
            new Translation2d(halfWheelbaseMeters, halfTrackWidthMeters),   // Front Left
            new Translation2d(halfWheelbaseMeters, -halfTrackWidthMeters),  // Front Right
            new Translation2d(-halfWheelbaseMeters, halfTrackWidthMeters),  // Back Left
            new Translation2d(-halfWheelbaseMeters, -halfTrackWidthMeters)  // Back Right
            );

    m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            m_gyroInputs.yawAngle,
            getModulePositions());
  }

  /**
   * Drives the robot with given translational and rotational velocities.
   *
   * @param xSpeedMetersPerSecond Forward/backward speed in meters per second.
   * @param ySpeedMetersPerSecond Left/right speed in meters per second.
   * @param rotationRadiansPerSecond Angular rotation rate in radians per second.
   * @param fieldRelative True if driving relative to the field, false for robot-relative.
   */
  public void drive(
      double xSpeedMetersPerSecond,
      double ySpeedMetersPerSecond,
      double rotationRadiansPerSecond,
      boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedMetersPerSecond, ySpeedMetersPerSecond, rotationRadiansPerSecond, getHeading())
            : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rotationRadiansPerSecond);

    SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        targetStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      targetStates[moduleIndex].optimize(m_moduleInputs[moduleIndex].steerAngle);
      double targetVoltage =
          (targetStates[moduleIndex].speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond)
              * 12.0;
      m_moduleIOs[moduleIndex].setDriveVoltage(targetVoltage);
      m_moduleIOs[moduleIndex].setSteerAngle(targetStates[moduleIndex].angle);
    }
  }

  /**
   * Creates a command to drive the robot using joystick input suppliers.
   *
   * @param xSpeedSupplier Supplier for X axis input (-1.0 to 1.0).
   * @param ySpeedSupplier Supplier for Y axis input (-1.0 to 1.0).
   * @param rotationSupplier Supplier for rotation axis input (-1.0 to 1.0).
   * @param fieldRelative Whether driving should be field relative.
   * @return A command that continuously drives the robot.
   */
  public Command driveCommand(
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotationSupplier,
      boolean fieldRelative) {
    return Commands.run(
            () ->
                drive(
                    xSpeedSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond,
                    ySpeedSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond,
                    rotationSupplier.getAsDouble() * SwerveConstants.kMaxAngularSpeedRadiansPerSecond,
                    fieldRelative),
            this)
        .withName("SwerveDrive.drive");
  }

  /** Resets the heading (yaw) of the robot to 0 degrees. */
  public void resetHeading() {
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /** @return The current estimated robot pose on the field in meters. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** @return The current robot heading as a Rotation2d. */
  public Rotation2d getHeading() {
    return m_gyroInputs.yawAngle;
  }

  /**
   * Resets the odometry to a specific pose.
   *
   * @param targetPose The new pose on the field.
   */
  public void resetOdometry(Pose2d targetPose) {
    m_odometry.resetPosition(getHeading(), getModulePositions(), targetPose);
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      positions[moduleIndex] =
          new SwerveModulePosition(
              m_moduleInputs[moduleIndex].drivePositionMeters,
              m_moduleInputs[moduleIndex].steerAngle);
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      states[moduleIndex] =
          new SwerveModuleState(
              m_moduleInputs[moduleIndex].driveVelocityMetersPerSecond,
              m_moduleInputs[moduleIndex].steerAngle);
    }
    return states;
  }

  @Override
  public void periodic() {
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      m_moduleIOs[moduleIndex].updateInputs(m_moduleInputs[moduleIndex]);
    }
    m_gyroIO.updateInputs(m_gyroInputs);

    if (m_gyroIO instanceof GyroIOSim) {
      ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
      Rotation2d newYawAngle =
          getHeading().plus(Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * 0.020));
      ((GyroIOSim) m_gyroIO).setYawAngle(newYawAngle);
    }

    m_odometry.update(getHeading(), getModulePositions());

    SmartDashboard.putNumber("Swerve/Pose X Meters", getPose().getX());
    SmartDashboard.putNumber("Swerve/Pose Y Meters", getPose().getY());
    SmartDashboard.putNumber("Swerve/Heading Degrees", getHeading().getDegrees());

    SwerveModuleState[] moduleStates = getModuleStates();
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      SmartDashboard.putNumber(
          "Swerve/Module " + moduleIndex + "/Speed (m per sec)",
          moduleStates[moduleIndex].speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Swerve/Module " + moduleIndex + "/Angle (deg)",
          moduleStates[moduleIndex].angle.getDegrees());
    }
  }
}
