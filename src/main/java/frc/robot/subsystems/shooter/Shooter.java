// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem managing:
 * 1. Flywheel (4x Kraken X60): Leader (CAN 35), Follower 1 (CAN 36), Follower 2 (CAN 40), Follower 3 (CAN 41)
 * 2. Hood (1x Kraken X60): CAN 37
 * 3. Supporting Shooter / Pre-roller (1x NEO Vortex on SPARK MAX): CAN 42
 *
 * <p>Total: strictly 6 motors (5 Kraken X60 + 1 NEO Vortex).
 */
public class Shooter extends SubsystemBase {
  private final ShooterIO m_shooterIO;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  private double m_targetFlywheelVelocityRotationsPerSecond = 0.0;
  private double m_targetHoodAngleRadians = 0.0;
  private double m_targetSupportingShooterVelocityRotationsPerSecond = 0.0;

  /**
   * Creates a new Shooter subsystem.
   *
   * @param shooterIO the hardware abstraction layer to use
   */
  public Shooter(ShooterIO shooterIO) {
    m_shooterIO = shooterIO;
  }

  // ─── Flywheel Control ──────────────────────────────────────────────────────

  /** Runs the shooter flywheel to default high-speed scoring velocity. */
  public void runFlywheel() {
    setFlywheelVelocity(ShooterConstants.kFlywheelTargetVelocityRotationsPerSecond);
  }

  /** Runs the flywheel at low idle speed to reduce spin-up latency. */
  public void runIdleFlywheel() {
    setFlywheelVelocity(ShooterConstants.kFlywheelIdleVelocityRotationsPerSecond);
  }

  /**
   * Sets closed-loop target velocity in rotations per second for the flywheel.
   *
   * @param velocityRotationsPerSecond target speed
   */
  public void setFlywheelVelocity(double velocityRotationsPerSecond) {
    m_targetFlywheelVelocityRotationsPerSecond = velocityRotationsPerSecond;
    m_shooterIO.setFlywheelVelocity(velocityRotationsPerSecond);
  }

  /** Stops the flywheel motors. */
  public void stopFlywheel() {
    m_targetFlywheelVelocityRotationsPerSecond = 0.0;
    m_shooterIO.stopFlywheel();
  }

  /** Sets open-loop raw voltage to flywheel motors. */
  public void setFlywheelVoltage(double appliedVolts) {
    m_targetFlywheelVelocityRotationsPerSecond = 0.0;
    m_shooterIO.setFlywheelVoltage(appliedVolts);
  }

  // ─── Hood Control ──────────────────────────────────────────────────────────

  /**
   * Sets closed-loop target angle for the adjustable hood.
   *
   * @param angleRadians target angle in radians
   */
  public void setHoodAngle(double angleRadians) {
    m_targetHoodAngleRadians = angleRadians;
    m_shooterIO.setHoodAngle(angleRadians);
  }

  /** Sets hood to stowed angle (0 rad). */
  public void stowHood() {
    setHoodAngle(ShooterConstants.kHoodStowAngleRadians);
  }

  /** Stops the hood motor. */
  public void stopHood() {
    m_shooterIO.stopHood();
  }

  /** Sets open-loop raw voltage to hood motor. */
  public void setHoodVoltage(double appliedVolts) {
    m_shooterIO.setHoodVoltage(appliedVolts);
  }

  // ─── Supporting Shooter Control (1x NEO Vortex on SPARK MAX) ───────────────

  /** Runs the supporting shooter kicker roller at default target feeding velocity. */
  public void runSupportingShooter() {
    setSupportingShooterVelocity(
        ShooterConstants.kSupportingShooterTargetVelocityRotationsPerSecond);
  }

  /**
   * Sets closed-loop target velocity for the supporting shooter.
   *
   * @param velocityRotationsPerSecond target speed
   */
  public void setSupportingShooterVelocity(double velocityRotationsPerSecond) {
    m_targetSupportingShooterVelocityRotationsPerSecond = velocityRotationsPerSecond;
    m_shooterIO.setSupportingShooterVelocity(velocityRotationsPerSecond);
  }

  /** Stops the supporting shooter motor. */
  public void stopSupportingShooter() {
    m_targetSupportingShooterVelocityRotationsPerSecond = 0.0;
    m_shooterIO.stopSupportingShooter();
  }

  /** Sets open-loop voltage to the supporting shooter motor. */
  public void setSupportingShooterVoltage(double appliedVolts) {
    m_targetSupportingShooterVelocityRotationsPerSecond = 0.0;
    m_shooterIO.setSupportingShooterVoltage(appliedVolts);
  }

  // ─── Combined Control ──────────────────────────────────────────────────────

  /** Prepares shot with specific flywheel velocity, hood angle, and starts supporting shooter. */
  public void prepareShot(double flywheelVelocityRps, double hoodAngleRad) {
    setFlywheelVelocity(flywheelVelocityRps);
    setHoodAngle(hoodAngleRad);
    runSupportingShooter();
  }

  /**
   * Automatically calculates and applies flywheel speed and hood angle from distance.
   *
   * @param distanceMeters distance to target in meters
   */
  public void setAimFromDistance(double distanceMeters) {
    double flywheelRps = frc.robot.util.AutoAim.calculateFlywheelVelocity(distanceMeters);
    double hoodAngleRad = frc.robot.util.AutoAim.calculateHoodAngle(distanceMeters);
    prepareShot(flywheelRps, hoodAngleRad);
  }

  /** Stops all shooter components (flywheel, hood, and supporting shooter). */
  public void stop() {
    stopFlywheel();
    stopHood();
    stopSupportingShooter();
  }

  // ─── Status & Getters ──────────────────────────────────────────────────────

  /** @return current flywheel velocity in rotations per second */
  public double getFlywheelVelocityRotationsPerSecond() {
    return m_inputs.flywheelVelocityRotationsPerSecond;
  }

  /** @return target flywheel velocity in rotations per second */
  public double getTargetFlywheelVelocityRotationsPerSecond() {
    return m_targetFlywheelVelocityRotationsPerSecond;
  }

  /** @return current hood angle in radians */
  public double getHoodAngleRadians() {
    return m_inputs.hoodAngleRadians;
  }

  /** @return target hood angle in radians */
  public double getTargetHoodAngleRadians() {
    return m_targetHoodAngleRadians;
  }

  /** @return current supporting shooter velocity in rotations per second */
  public double getSupportingShooterVelocityRotationsPerSecond() {
    return m_inputs.supportingShooterVelocityRotationsPerSecond;
  }

  /** @return target supporting shooter velocity in rotations per second */
  public double getTargetSupportingShooterVelocityRotationsPerSecond() {
    return m_targetSupportingShooterVelocityRotationsPerSecond;
  }

  /** @return true when flywheel is within velocity tolerance */
  public boolean atTargetFlywheelSpeed() {
    if (m_targetFlywheelVelocityRotationsPerSecond <= 0.0) {
      return false;
    }
    return Math.abs(
            m_inputs.flywheelVelocityRotationsPerSecond - m_targetFlywheelVelocityRotationsPerSecond)
        <= ShooterConstants.kFlywheelToleranceRotationsPerSecond;
  }

  /** @return true when hood is within angle tolerance */
  public boolean atTargetHoodAngle() {
    return Math.abs(m_inputs.hoodAngleRadians - m_targetHoodAngleRadians)
        <= ShooterConstants.kHoodToleranceRadians;
  }

  /** @return true when supporting shooter is within velocity tolerance */
  public boolean atTargetSupportingShooterSpeed() {
    if (m_targetSupportingShooterVelocityRotationsPerSecond <= 0.0) {
      return false;
    }
    return Math.abs(
            m_inputs.supportingShooterVelocityRotationsPerSecond
                - m_targetSupportingShooterVelocityRotationsPerSecond)
        <= ShooterConstants.kSupportingShooterToleranceRotationsPerSecond;
  }

  /** @return true when flywheel, hood, and supporting shooter have reached their target setpoints */
  public boolean isReadyToShoot() {
    boolean flywheelReady = atTargetFlywheelSpeed();
    boolean hoodReady = atTargetHoodAngle();
    boolean supportingReady =
        m_targetSupportingShooterVelocityRotationsPerSecond <= 0.0
            || atTargetSupportingShooterSpeed();
    return flywheelReady && hoodReady && supportingReady;
  }

  // ─── Command Factories ─────────────────────────────────────────────────────

  /** @return command that spools the shooter and positions hood for shooting */
  public Command prepareShootCommand(double flywheelVelocityRps, double hoodAngleRad) {
    return Commands.startEnd(
            () -> prepareShot(flywheelVelocityRps, hoodAngleRad),
            this::stop,
            this)
        .withName("Shooter.prepareShot");
  }

  /** @return command that stops all shooter components */
  public Command stopCommand() {
    return Commands.runOnce(this::stop, this).withName("Shooter.stop");
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    m_shooterIO.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);

    // AdvantageKit Telemetry
    Logger.recordOutput("Shooter/FlywheelVelocityRps", m_inputs.flywheelVelocityRotationsPerSecond);
    Logger.recordOutput("Shooter/FlywheelTargetVelocityRps", m_targetFlywheelVelocityRotationsPerSecond);
    Logger.recordOutput("Shooter/FlywheelAppliedOutputVolts", m_inputs.flywheelAppliedVolts);
    Logger.recordOutput("Shooter/FlywheelLeaderCurrentAmps", m_inputs.flywheelLeaderCurrentAmps);
    Logger.recordOutput("Shooter/FlywheelFollower1CurrentAmps", m_inputs.flywheelFollower1CurrentAmps);
    Logger.recordOutput("Shooter/FlywheelFollower2CurrentAmps", m_inputs.flywheelFollower2CurrentAmps);
    Logger.recordOutput("Shooter/FlywheelFollower3CurrentAmps", m_inputs.flywheelFollower3CurrentAmps);
    Logger.recordOutput("Shooter/FlywheelAtTargetSpeed", atTargetFlywheelSpeed());

    Logger.recordOutput("Shooter/HoodAngleDegrees", Math.toDegrees(m_inputs.hoodAngleRadians));
    Logger.recordOutput("Shooter/HoodTargetAngleDegrees", Math.toDegrees(m_targetHoodAngleRadians));
    Logger.recordOutput("Shooter/HoodAppliedOutputVolts", m_inputs.hoodAppliedVolts);
    Logger.recordOutput("Shooter/HoodCurrentAmps", m_inputs.hoodCurrentAmps);
    Logger.recordOutput("Shooter/HoodAtTargetAngle", atTargetHoodAngle());

    Logger.recordOutput(
        "Shooter/SupportingVelocityRps", m_inputs.supportingShooterVelocityRotationsPerSecond);
    Logger.recordOutput(
        "Shooter/SupportingTargetVelocityRps", m_targetSupportingShooterVelocityRotationsPerSecond);
    Logger.recordOutput("Shooter/SupportingAppliedOutputVolts", m_inputs.supportingShooterAppliedVolts);
    Logger.recordOutput("Shooter/SupportingCurrentAmps", m_inputs.supportingShooterCurrentAmps);
    Logger.recordOutput("Shooter/SupportingAtSpeed", atTargetSupportingShooterSpeed());
    Logger.recordOutput("Shooter/ReadyToShoot", isReadyToShoot());

    // Flywheel Telemetry (4 Krakens)
    SmartDashboard.putNumber("Shooter/Flywheel Velocity (RPS)", m_inputs.flywheelVelocityRotationsPerSecond);
    SmartDashboard.putNumber("Shooter/Flywheel Target (RPS)", m_targetFlywheelVelocityRotationsPerSecond);
    SmartDashboard.putNumber("Shooter/Flywheel Output (V)", m_inputs.flywheelAppliedVolts);
    SmartDashboard.putNumber("Shooter/Flywheel Leader Current (A)", m_inputs.flywheelLeaderCurrentAmps);
    SmartDashboard.putNumber("Shooter/Flywheel Follower 1 Current (A)", m_inputs.flywheelFollower1CurrentAmps);
    SmartDashboard.putNumber("Shooter/Flywheel Follower 2 Current (A)", m_inputs.flywheelFollower2CurrentAmps);
    SmartDashboard.putNumber("Shooter/Flywheel Follower 3 Current (A)", m_inputs.flywheelFollower3CurrentAmps);
    SmartDashboard.putBoolean("Shooter/At Target Speed", atTargetFlywheelSpeed());

    // Hood Telemetry (1 Kraken X60 CAN 37)
    SmartDashboard.putNumber("Shooter/Hood Angle (deg)", Math.toDegrees(m_inputs.hoodAngleRadians));
    SmartDashboard.putNumber("Shooter/Hood Target (deg)", Math.toDegrees(m_targetHoodAngleRadians));
    SmartDashboard.putNumber("Shooter/Hood Output (V)", m_inputs.hoodAppliedVolts);
    SmartDashboard.putNumber("Shooter/Hood Current (A)", m_inputs.hoodCurrentAmps);
    SmartDashboard.putBoolean("Shooter/Hood At Target Angle", atTargetHoodAngle());

    // Supporting Shooter Telemetry (1 NEO Vortex CAN 42)
    SmartDashboard.putNumber(
        "Shooter/Supporting Velocity (RPS)", m_inputs.supportingShooterVelocityRotationsPerSecond);
    SmartDashboard.putNumber(
        "Shooter/Supporting Target (RPS)", m_targetSupportingShooterVelocityRotationsPerSecond);
    SmartDashboard.putNumber("Shooter/Supporting Output (V)", m_inputs.supportingShooterAppliedVolts);
    SmartDashboard.putNumber("Shooter/Supporting Current (A)", m_inputs.supportingShooterCurrentAmps);
    SmartDashboard.putBoolean("Shooter/Supporting At Speed", atTargetSupportingShooterSpeed());

    // Combined Ready State
    SmartDashboard.putBoolean("Shooter/Ready To Shoot", isReadyToShoot());
  }
}
