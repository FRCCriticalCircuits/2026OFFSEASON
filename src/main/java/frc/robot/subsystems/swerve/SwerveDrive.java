// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.SwerveConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveDrive extends SubsystemBase {
  private final GyroIO m_gyroIO;
  private final GyroIO.GyroIOInputs m_gyroInputs = new GyroIO.GyroIOInputs();

  private final SwerveModuleIO[] m_moduleIOs;
  private final SwerveModuleIO.SwerveModuleIOInputs[] m_moduleInputs;

  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;

  private final PIDController m_headingController =
      new PIDController(
          AutoAimConstants.kHeadingProportionalGain,
          AutoAimConstants.kHeadingIntegralGain,
          AutoAimConstants.kHeadingDerivativeGain);

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

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);
    m_headingController.setTolerance(AutoAimConstants.kHeadingToleranceRadians);

    configureAutoBuilder();
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
   * Drives with manual translation control while automatically locking heading onto a target angle.
   *
   * @param xSpeedMetersPerSecond Forward/backward velocity in meters per second.
   * @param ySpeedMetersPerSecond Left/right velocity in meters per second.
   * @param targetHeading Target heading as a Rotation2d.
   * @param fieldRelative True if translation is field-relative.
   */
  public void driveWithHeadingLock(
      double xSpeedMetersPerSecond,
      double ySpeedMetersPerSecond,
      Rotation2d targetHeading,
      boolean fieldRelative) {
    double rotationOutput =
        m_headingController.calculate(getHeading().getRadians(), targetHeading.getRadians());
    drive(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rotationOutput, fieldRelative);
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

  /**
   * Creates an auto-aim drive command that rotates to track a target heading while the driver translates.
   *
   * @param xSpeedSupplier Supplier for X translation.
   * @param ySpeedSupplier Supplier for Y translation.
   * @param targetHeadingSupplier Supplier providing the target heading.
   * @return An auto-aim drive command.
   */
  public Command autoAimDriveCommand(
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      Supplier<Rotation2d> targetHeadingSupplier) {
    return Commands.run(
            () -> {
              double autoAimMaxSpeedMetersPerSecond =
                  SwerveConstants.kMaxSpeedMetersPerSecond * AutoAimConstants.kAutoAimMaxSpeedMultiplier;
              driveWithHeadingLock(
                  xSpeedSupplier.getAsDouble() * autoAimMaxSpeedMetersPerSecond,
                  ySpeedSupplier.getAsDouble() * autoAimMaxSpeedMetersPerSecond,
                  targetHeadingSupplier.get(),
                  true);
            },
            this)
        .withName("SwerveDrive.autoAimDrive");
  }

  /** Checks if the robot's heading is within tolerance of a target heading. */
  public boolean isHeadingAligned(Rotation2d targetHeading) {
    double angleDiffRadians = Math.abs(getHeading().minus(targetHeading).getRadians());
    return angleDiffRadians <= AutoAimConstants.kHeadingToleranceRadians;
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

  /**
   * Returns the current robot-relative chassis speeds.
   *
   * @return Current robot-relative ChassisSpeeds.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Drives the robot using robot-relative chassis speeds.
   *
   * @param speeds Target robot-relative ChassisSpeeds.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(speeds);
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
   * Returns the swerve drive kinematics object.
   *
   * @return Kinematics instance for this drivetrain.
   */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Configures PathPlanner AutoBuilder for autonomous trajectory tracking.
   */
  public void configureAutoBuilder() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      double halfWheelbaseMeters = SwerveConstants.kWheelbaseMeters / 2.0;
      double halfTrackWidthMeters = SwerveConstants.kTrackWidthMeters / 2.0;
      Translation2d[] moduleOffsets =
          new Translation2d[] {
            new Translation2d(halfWheelbaseMeters, halfTrackWidthMeters),
            new Translation2d(halfWheelbaseMeters, -halfTrackWidthMeters),
            new Translation2d(-halfWheelbaseMeters, halfTrackWidthMeters),
            new Translation2d(-halfWheelbaseMeters, -halfTrackWidthMeters)
          };
      ModuleConfig moduleConfig =
          new ModuleConfig(
              SwerveConstants.kWheelRadiusMeters,
              SwerveConstants.kMaxSpeedMetersPerSecond,
              SwerveConstants.kWheelCOF,
              DCMotor.getKrakenX60(1),
              SwerveConstants.kDriveGearRatio,
              SwerveConstants.kDriveSupplyCurrentLimitAmps,
              1);
      robotConfig =
          new RobotConfig(
              SwerveConstants.kRobotMassKg,
              SwerveConstants.kRobotMOIKgM2,
              moduleConfig,
              moduleOffsets);
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new PPHolonomicDriveController(
            new PIDConstants(
                SwerveConstants.kAutoTranslationKP,
                SwerveConstants.kAutoTranslationKI,
                SwerveConstants.kAutoTranslationKD),
            new PIDConstants(
                SwerveConstants.kAutoRotationKP,
                SwerveConstants.kAutoRotationKI,
                SwerveConstants.kAutoRotationKD)),
        robotConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
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
