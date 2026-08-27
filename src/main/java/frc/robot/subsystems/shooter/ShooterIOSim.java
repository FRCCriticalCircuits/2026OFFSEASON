// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants;

/**
 * WPILib simulation implementation of {@link ShooterIO} supporting flywheel and hood plants.
 */
public class ShooterIOSim implements ShooterIO {
  // ── Flywheel Simulation ───────────────────────────────────────────────────
  private final FlywheelSim m_flywheelSimulation =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(2), 0.003, ShooterConstants.kFlywheelGearRatio),
          DCMotor.getKrakenX60(2));

  private final PIDController m_flywheelFeedback =
      new PIDController(ShooterConstants.kFlywheelProportionalGain, 0.0, 0.0);

  private double m_flywheelTargetVelocityRotationsPerSecond = 0.0;
  private double m_flywheelAppliedVolts = 0.0;
  private boolean m_flywheelClosedLoop = false;

  // ── Hood Simulation ───────────────────────────────────────────────────────
  private final SingleJointedArmSim m_hoodSimulation =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          ShooterConstants.kHoodGearRatio,
          0.005, // J (kg*m^2)
          0.20,  // arm length (m)
          ShooterConstants.kHoodMinAngleRadians,
          ShooterConstants.kHoodMaxAngleRadians,
          false, // simulate gravity (hood is spring/linkage constrained)
          0.0    // starting angle (rad)
      );

  private final PIDController m_hoodFeedback =
      new PIDController(ShooterConstants.kHoodProportionalGain, 0.0, ShooterConstants.kHoodDerivativeGain);

  private double m_hoodTargetAngleRadians = 0.0;
  private double m_hoodAppliedVolts = 0.0;
  private boolean m_hoodClosedLoop = false;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // 1. Flywheel Sim Update
    if (m_flywheelClosedLoop && m_flywheelTargetVelocityRotationsPerSecond > 0.0) {
      double currentRps = m_flywheelSimulation.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
      double feedforwardVolts =
          (m_flywheelTargetVelocityRotationsPerSecond * ShooterConstants.kFlywheelVelocityGain)
              + ShooterConstants.kFlywheelStaticGain;
      double feedbackVolts =
          m_flywheelFeedback.calculate(currentRps, m_flywheelTargetVelocityRotationsPerSecond);
      m_flywheelAppliedVolts = MathUtil.clamp(feedforwardVolts + feedbackVolts, -12.0, 12.0);
    }

    m_flywheelSimulation.setInputVoltage(m_flywheelAppliedVolts);
    m_flywheelSimulation.update(0.020);

    double currentFlywheelRps = m_flywheelSimulation.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.flywheelVelocityRotationsPerSecond = currentFlywheelRps;
    inputs.flywheelTargetVelocityRotationsPerSecond = m_flywheelTargetVelocityRotationsPerSecond;
    inputs.flywheelAppliedVolts = m_flywheelAppliedVolts;
    inputs.flywheelLeaderCurrentAmps = m_flywheelSimulation.getCurrentDrawAmps() / 2.0;
    inputs.flywheelFollowerCurrentAmps = m_flywheelSimulation.getCurrentDrawAmps() / 2.0;

    // 2. Hood Sim Update
    if (m_hoodClosedLoop) {
      double currentAngleRad = m_hoodSimulation.getAngleRads();
      m_hoodAppliedVolts =
          MathUtil.clamp(
              m_hoodFeedback.calculate(currentAngleRad, m_hoodTargetAngleRadians),
              -12.0,
              12.0);
    }

    m_hoodSimulation.setInputVoltage(m_hoodAppliedVolts);
    m_hoodSimulation.update(0.020);

    inputs.hoodAngleRadians = m_hoodSimulation.getAngleRads();
    inputs.hoodTargetAngleRadians = m_hoodTargetAngleRadians;
    inputs.hoodAppliedVolts = m_hoodAppliedVolts;
    inputs.hoodCurrentAmps = m_hoodSimulation.getCurrentDrawAmps();
  }

  // ── Flywheel Control ───────────────────────────────────────────────────────

  @Override
  public void setFlywheelVelocity(double velocityRotationsPerSecond) {
    m_flywheelClosedLoop = true;
    m_flywheelTargetVelocityRotationsPerSecond = velocityRotationsPerSecond;
  }

  @Override
  public void setFlywheelVoltage(double appliedVolts) {
    m_flywheelClosedLoop = false;
    m_flywheelTargetVelocityRotationsPerSecond = 0.0;
    m_flywheelAppliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void stopFlywheel() {
    m_flywheelClosedLoop = false;
    m_flywheelTargetVelocityRotationsPerSecond = 0.0;
    m_flywheelAppliedVolts = 0.0;
  }

  // ── Hood Control ───────────────────────────────────────────────────────────

  @Override
  public void setHoodAngle(double angleRadians) {
    m_hoodClosedLoop = true;
    m_hoodTargetAngleRadians =
        MathUtil.clamp(
            angleRadians,
            ShooterConstants.kHoodMinAngleRadians,
            ShooterConstants.kHoodMaxAngleRadians);
  }

  @Override
  public void setHoodVoltage(double appliedVolts) {
    m_hoodClosedLoop = false;
    m_hoodAppliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void stopHood() {
    m_hoodClosedLoop = false;
    m_hoodAppliedVolts = 0.0;
  }

  @Override
  public void resetHoodEncoder() {
    // Sim reset handled internally
  }
}
