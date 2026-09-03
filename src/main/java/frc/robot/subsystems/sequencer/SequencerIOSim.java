// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sequencer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SequencerConstants;

/**
 * WPILib simulation implementation of {@link SequencerIO} using {@link FlywheelSim}.
 */
public class SequencerIOSim implements SequencerIO {
  private final FlywheelSim m_sequencerSimulation =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNeoVortex(2), 0.001, SequencerConstants.kGearRatio),
          DCMotor.getNeoVortex(2));

  private double m_appliedVolts = 0.0;
  private double m_targetVelocityRotationsPerSecond = 0.0;
  private boolean m_velocityControl = false;

  @Override
  public void updateInputs(SequencerIOInputs inputs) {
    if (m_velocityControl) {
      // Simulate feedforward kV voltage
      m_appliedVolts =
          MathUtil.clamp(
              m_targetVelocityRotationsPerSecond * SequencerConstants.kVelocityGain
                  + Math.signum(m_targetVelocityRotationsPerSecond) * SequencerConstants.kStaticGain,
              -12.0,
              12.0);
    }

    m_sequencerSimulation.setInputVoltage(m_appliedVolts);
    m_sequencerSimulation.update(0.020);

    inputs.velocityRotationsPerSecond =
        m_sequencerSimulation.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.positionRotations += inputs.velocityRotationsPerSecond * 0.020;
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_sequencerSimulation.getCurrentDrawAmps();
    inputs.leaderCurrentAmps = inputs.currentAmps / 2.0;
    inputs.followerCurrentAmps = inputs.currentAmps / 2.0;
  }

  @Override
  public void setVelocity(double velocityRotationsPerSecond) {
    m_velocityControl = true;
    m_targetVelocityRotationsPerSecond = velocityRotationsPerSecond;
  }

  @Override
  public void setVoltage(double appliedVolts) {
    m_velocityControl = false;
    m_appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    m_velocityControl = false;
    m_appliedVolts = 0.0;
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {}
}
