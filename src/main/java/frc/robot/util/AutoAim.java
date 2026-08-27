// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoAimConstants;

/**
 * Auto-Aim calculation engine for dynamic distance, robot heading alignment,
 * flywheel velocity, and adjustable hood angle calculations.
 */
public class AutoAim {

  private static final InterpolatingDoubleTreeMap m_flywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_hoodAngleMap = new InterpolatingDoubleTreeMap();

  static {
    // ── Distance (m) -> Flywheel Velocity (rotations per second) ────────────
    // TODO: Calibrate these empirical points on your physical robot
    m_flywheelSpeedMap.put(1.5, 55.0);
    m_flywheelSpeedMap.put(2.5, 62.0);
    m_flywheelSpeedMap.put(3.5, 70.0);
    m_flywheelSpeedMap.put(4.5, 78.0);
    m_flywheelSpeedMap.put(5.5, 86.0);
    m_flywheelSpeedMap.put(6.5, 94.0);

    // ── Distance (m) -> Hood Angle (radians) ────────────────────────────────
    // TODO: Calibrate these empirical points on your physical robot
    m_hoodAngleMap.put(1.5, Math.toRadians(12.0));
    m_hoodAngleMap.put(2.5, Math.toRadians(20.0));
    m_hoodAngleMap.put(3.5, Math.toRadians(27.0));
    m_hoodAngleMap.put(4.5, Math.toRadians(33.0));
    m_hoodAngleMap.put(5.5, Math.toRadians(38.0));
    m_hoodAngleMap.put(6.5, Math.toRadians(42.0));
  }

  /** Result container for Auto-Aim calculations. */
  public static class AutoAimResult {
    public final double distanceMeters;
    public final Rotation2d targetHeading;
    public final double flywheelVelocityRotationsPerSecond;
    public final double hoodAngleRadians;
    public final boolean headingAligned;
    public final boolean distanceInRange;

    public AutoAimResult(
        double distanceMeters,
        Rotation2d targetHeading,
        double flywheelVelocityRotationsPerSecond,
        double hoodAngleRadians,
        boolean headingAligned,
        boolean distanceInRange) {
      this.distanceMeters = distanceMeters;
      this.targetHeading = targetHeading;
      this.flywheelVelocityRotationsPerSecond = flywheelVelocityRotationsPerSecond;
      this.hoodAngleRadians = hoodAngleRadians;
      this.headingAligned = headingAligned;
      this.distanceInRange = distanceInRange;
    }
  }

  /**
   * Retrieves active alliance goal coordinate on the field.
   *
   * @return translation of the target goal
   */
  public static Translation2d getTargetGoalLocation() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRed ? AutoAimConstants.kRedGoalLocation : AutoAimConstants.kBlueGoalLocation;
  }

  /**
   * Calculates distance in meters from robot pose to active target goal.
   *
   * @param robotPose current robot field pose
   * @return distance in meters
   */
  public static double calculateDistanceToGoal(Pose2d robotPose) {
    return robotPose.getTranslation().getDistance(getTargetGoalLocation());
  }

  /**
   * Calculates the target field-relative heading angle the robot must face to point at the goal.
   *
   * @param robotPose current robot field pose
   * @return desired robot heading
   */
  public static Rotation2d calculateTargetHeading(Pose2d robotPose) {
    Translation2d targetGoal = getTargetGoalLocation();
    double deltaX = targetGoal.getX() - robotPose.getX();
    double deltaY = targetGoal.getY() - robotPose.getY();
    return new Rotation2d(Math.atan2(deltaY, deltaX));
  }

  /**
   * Evaluates the interpolated flywheel velocity for the given distance.
   *
   * @param distanceMeters distance to goal in meters
   * @return flywheel velocity in rotations per second
   */
  public static double calculateFlywheelVelocity(double distanceMeters) {
    return m_flywheelSpeedMap.get(distanceMeters);
  }

  /**
   * Evaluates the interpolated hood angle for the given distance.
   *
   * @param distanceMeters distance to goal in meters
   * @return hood angle in radians
   */
  public static double calculateHoodAngle(double distanceMeters) {
    return m_hoodAngleMap.get(distanceMeters);
  }

  /**
   * Performs full Auto-Aim calculation given the current robot pose.
   *
   * @param robotPose current robot field pose
   * @return calculated auto-aim targets and alignment status
   */
  public static AutoAimResult calculate(Pose2d robotPose) {
    double distance = calculateDistanceToGoal(robotPose);
    Rotation2d targetHeading = calculateTargetHeading(robotPose);

    double flywheelRps = calculateFlywheelVelocity(distance);
    double hoodAngleRad = calculateHoodAngle(distance);

    double angleErrorRadians = Math.abs(robotPose.getRotation().minus(targetHeading).getRadians());
    boolean headingAligned = angleErrorRadians <= AutoAimConstants.kHeadingToleranceRadians;

    boolean distanceInRange =
        distance >= AutoAimConstants.kMinDistanceMeters
            && distance <= AutoAimConstants.kMaxDistanceMeters;

    return new AutoAimResult(
        distance,
        targetHeading,
        flywheelRps,
        hoodAngleRad,
        headingAligned,
        distanceInRange);
  }
}
