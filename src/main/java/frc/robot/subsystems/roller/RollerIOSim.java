// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RollerConstants;

/**
 * WPILib simulation implementation of {@link RollerIO} modeling 2 Kraken X60 motors.
 */
public class RollerIOSim implements RollerIO {
  private final FlywheelSim m_rollerSimulation =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(2), 0.001, RollerConstants.kGearRatio),
          DCMotor.getKrakenX60(2));

  private double m_appliedVolts = 0.0;

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (inputs == null) {
      return;
    }
    m_rollerSimulation.setInputVoltage(m_appliedVolts);
    m_rollerSimulation.update(0.020);

    // Convert rad/s -> rotations/s
    inputs.velocityRotationsPerSecond =
        m_rollerSimulation.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.appliedVolts = m_appliedVolts;
    double totalCurrent = m_rollerSimulation.getCurrentDrawAmps();
    inputs.leaderCurrentAmps = totalCurrent / 2.0;
    inputs.followerCurrentAmps = totalCurrent / 2.0;
    inputs.currentAmps = inputs.leaderCurrentAmps;
    inputs.gamePieceDetected = false;
  }

  @Override
  public void setVoltage(double appliedVolts) {
    if (!Double.isFinite(appliedVolts)) {
      m_appliedVolts = 0.0;
      return;
    }
    m_appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    // Hardware-only concept
  }
}
