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
 * WPILib simulation implementation of {@link RollerIO}.
 */
public class RollerIOSim implements RollerIO {
  private final FlywheelSim m_rollerSimulation =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), 0.001, RollerConstants.kGearRatio),
          DCMotor.getKrakenX60(1));

  private double m_appliedVolts = 0.0;

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    m_rollerSimulation.setInputVoltage(m_appliedVolts);
    m_rollerSimulation.update(0.020);

    // Convert rad/s -> rotations/s
    inputs.velocityRotationsPerSecond =
        m_rollerSimulation.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_rollerSimulation.getCurrentDrawAmps();
    inputs.gamePieceDetected = false;
  }

  @Override
  public void setVoltage(double appliedVolts) {
    m_appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    // Hardware-only concept
  }
}
