// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private final FlywheelSim m_driveSimulation;
  private final FlywheelSim m_steerSimulation;

  private double m_driveAppliedVolts = 0.0;
  private double m_steerAppliedVolts = 0.0;

  private double m_drivePositionMeters = 0.0;
  private double m_steerAngleRadians = 0.0;

  public SwerveModuleIOSim() {
    m_driveSimulation =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1), 0.025, SwerveConstants.kDriveGearRatio),
            DCMotor.getKrakenX60(1));
    m_steerSimulation =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX44(1), 0.004, SwerveConstants.kSteerGearRatio),
            DCMotor.getKrakenX44(1));
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    m_driveSimulation.update(0.020);
    m_steerSimulation.update(0.020);

    double driveVelocityRadiansPerSecond = m_driveSimulation.getAngularVelocityRadPerSec();
    double steerVelocityRadiansPerSecond = m_steerSimulation.getAngularVelocityRadPerSec();

    m_drivePositionMeters += (driveVelocityRadiansPerSecond * 0.020) * SwerveConstants.kWheelRadiusMeters;
    m_steerAngleRadians += steerVelocityRadiansPerSecond * 0.020;

    inputs.drivePositionMeters = m_drivePositionMeters;
    inputs.driveVelocityMetersPerSecond =
        driveVelocityRadiansPerSecond * SwerveConstants.kWheelRadiusMeters;
    inputs.driveAppliedVolts = m_driveAppliedVolts;
    inputs.driveCurrentAmps = m_driveSimulation.getCurrentDrawAmps();

    inputs.steerAngle = new Rotation2d(m_steerAngleRadians);
    inputs.steerVelocityRadiansPerSecond = steerVelocityRadiansPerSecond;
    inputs.steerAppliedVolts = m_steerAppliedVolts;
    inputs.steerCurrentAmps = m_steerSimulation.getCurrentDrawAmps();
  }

  @Override
  public void setDriveVoltage(double appliedVolts) {
    m_driveAppliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_driveSimulation.setInputVoltage(m_driveAppliedVolts);
  }

  @Override
  public void setSteerAngle(Rotation2d steerAngle) {
    // Proportional controller for simulated steering response
    double steerAngleErrorRadians = steerAngle.getRadians() - m_steerAngleRadians;
    m_steerAppliedVolts = MathUtil.clamp(steerAngleErrorRadians * 10.0, -12.0, 12.0);
    m_steerSimulation.setInputVoltage(m_steerAppliedVolts);
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    // Hardware-only concept
  }
}
