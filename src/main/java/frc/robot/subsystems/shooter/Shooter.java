// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Combined Shooter subsystem managing the 4-Kraken Flywheel, 2-NEO-Vortex Accelerator, and adjustable Hood.
 */
public class Shooter extends SubsystemBase {
  private final ShooterIO m_shooterIO;
  private final ShooterIO.ShooterIOInputs m_inputs = new ShooterIO.ShooterIOInputs();

  private double m_targetFlywheelVelocityRotationsPerSecond = 0.0;
  private double m_targetAcceleratorVelocityRotationsPerSecond = 0.0;
  private double m_targetHoodAngleRadians = 0.0;

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

  // ─── Accelerator / Kicker Control ──────────────────────────────────────────

  /** Runs the accelerator kicker roller at default target feeding velocity. */
  public void runAccelerator() {
    setAcceleratorVelocity(ShooterConstants.kAcceleratorTargetVelocityRotationsPerSecond);
  }

  /**
   * Sets closed-loop target velocity for the accelerator.
   *
   * @param velocityRotationsPerSecond target speed
   */
  public void setAcceleratorVelocity(double velocityRotationsPerSecond) {
    m_targetAcceleratorVelocityRotationsPerSecond = velocityRotationsPerSecond;
    m_shooterIO.setAcceleratorVelocity(velocityRotationsPerSecond);
  }

  /** Stops the accelerator motors. */
  public void stopAccelerator() {
    m_targetAcceleratorVelocityRotationsPerSecond = 0.0;
    m_shooterIO.stopAccelerator();
  }

  /** Sets open-loop voltage to the accelerator motors. */
  public void setAcceleratorVoltage(double appliedVolts) {
    m_targetAcceleratorVelocityRotationsPerSecond = 0.0;
    m_shooterIO.setAcceleratorVoltage(appliedVolts);
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

  // ─── Combined Control ──────────────────────────────────────────────────────

  /** Prepares shot with specific flywheel velocity and hood angle, spinning up the accelerator as well. */
  public void prepareShot(double flywheelVelocityRps, double hoodAngleRad) {
    setFlywheelVelocity(flywheelVelocityRps);
    setHoodAngle(hoodAngleRad);
    runAccelerator();
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

  /** Stops all shooter components (flywheel, hood, and accelerator). */
  public void stop() {
    stopFlywheel();
    stopHood();
    stopAccelerator();
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

  /** @return current accelerator velocity in rotations per second */
  public double getAcceleratorVelocityRotationsPerSecond() {
    return m_inputs.acceleratorVelocityRotationsPerSecond;
  }

  /** @return target accelerator velocity in rotations per second */
  public double getTargetAcceleratorVelocityRotationsPerSecond() {
    return m_targetAcceleratorVelocityRotationsPerSecond;
  }

  /** @return current hood angle in radians */
  public double getHoodAngleRadians() {
    return m_inputs.hoodAngleRadians;
  }

  /** @return target hood angle in radians */
  public double getTargetHoodAngleRadians() {
    return m_targetHoodAngleRadians;
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

  /** @return true when accelerator is within velocity tolerance */
  public boolean atTargetAcceleratorSpeed() {
    if (m_targetAcceleratorVelocityRotationsPerSecond <= 0.0) {
      return false;
    }
    return Math.abs(
            m_inputs.acceleratorVelocityRotationsPerSecond - m_targetAcceleratorVelocityRotationsPerSecond)
        <= ShooterConstants.kAcceleratorToleranceRotationsPerSecond;
  }

  /** @return true when hood is within angle tolerance */
  public boolean atTargetHoodAngle() {
    return Math.abs(m_inputs.hoodAngleRadians - m_targetHoodAngleRadians)
        <= ShooterConstants.kHoodToleranceRadians;
  }

  /** @return true when flywheel, hood, and accelerator have reached their target setpoints */
  public boolean isReadyToShoot() {
    boolean flywheelReady = atTargetFlywheelSpeed();
    boolean hoodReady = atTargetHoodAngle();
    boolean accelReady =
        m_targetAcceleratorVelocityRotationsPerSecond <= 0.0 || atTargetAcceleratorSpeed();
    return flywheelReady && hoodReady && accelReady;
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

    // Flywheel Telemetry (4 Krakens)
    SmartDashboard.putNumber("Shooter/Flywheel Velocity (RPS)", m_inputs.flywheelVelocityRotationsPerSecond);
    SmartDashboard.putNumber("Shooter/Flywheel Target (RPS)", m_targetFlywheelVelocityRotationsPerSecond);
    SmartDashboard.putNumber("Shooter/Flywheel Output (V)", m_inputs.flywheelAppliedVolts);
    SmartDashboard.putNumber("Shooter/Flywheel Leader Current (A)", m_inputs.flywheelLeaderCurrentAmps);
    SmartDashboard.putNumber("Shooter/Flywheel Follower 1 Current (A)", m_inputs.flywheelFollower1CurrentAmps);
    SmartDashboard.putNumber("Shooter/Flywheel Follower 2 Current (A)", m_inputs.flywheelFollower2CurrentAmps);
    SmartDashboard.putNumber("Shooter/Flywheel Follower 3 Current (A)", m_inputs.flywheelFollower3CurrentAmps);
    SmartDashboard.putBoolean("Shooter/At Target Speed", atTargetFlywheelSpeed());

    // Accelerator Telemetry (2 NEO Vortex)
    SmartDashboard.putNumber("Shooter/Accelerator Velocity (RPS)", m_inputs.acceleratorVelocityRotationsPerSecond);
    SmartDashboard.putNumber("Shooter/Accelerator Target (RPS)", m_targetAcceleratorVelocityRotationsPerSecond);
    SmartDashboard.putNumber("Shooter/Accelerator Output (V)", m_inputs.acceleratorAppliedVolts);
    SmartDashboard.putNumber("Shooter/Accelerator Leader Current (A)", m_inputs.acceleratorLeaderCurrentAmps);
    SmartDashboard.putNumber("Shooter/Accelerator Follower Current (A)", m_inputs.acceleratorFollowerCurrentAmps);
    SmartDashboard.putBoolean("Shooter/Accelerator At Speed", atTargetAcceleratorSpeed());

    // Hood Telemetry
    SmartDashboard.putNumber("Shooter/Hood Angle (deg)", Math.toDegrees(m_inputs.hoodAngleRadians));
    SmartDashboard.putNumber("Shooter/Hood Target (deg)", Math.toDegrees(m_targetHoodAngleRadians));
    SmartDashboard.putNumber("Shooter/Hood Output (V)", m_inputs.hoodAppliedVolts);
    SmartDashboard.putNumber("Shooter/Hood Current (A)", m_inputs.hoodCurrentAmps);
    SmartDashboard.putBoolean("Shooter/Hood At Target Angle", atTargetHoodAngle());

    // Combined Ready State
    SmartDashboard.putBoolean("Shooter/Ready To Shoot", isReadyToShoot());
  }
}
