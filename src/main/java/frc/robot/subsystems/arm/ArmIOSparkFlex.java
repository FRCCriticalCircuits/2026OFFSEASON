// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ArmConstants;

/**
 * Hardware IO implementation for the arm subsystem using a NEO Vortex motor driven by a SPARK Flex controller.
 */
public class ArmIOSparkFlex implements ArmIO {
  private final SparkFlex m_motor;
  private final RelativeEncoder m_encoder;

  /**
   * Constructs an ArmIOSparkFlex instance.
   *
   * @param motorCanId CAN ID of the arm SPARK Flex motor controller.
   */
  public ArmIOSparkFlex(int motorCanId) {
    m_motor = new SparkFlex(motorCanId, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();

    SparkFlexConfig configuration = new SparkFlexConfig();
    configuration.idleMode(IdleMode.kBrake);
    configuration.inverted(false);
    configuration.smartCurrentLimit(ArmConstants.kSmartCurrentLimitAmps);
    configuration.voltageCompensation(ArmConstants.kVoltageCompensationVolts);

    // Position factor: rotations -> mechanism radians (2π / gearRatio)
    configuration.encoder.positionConversionFactor((2.0 * Math.PI) / ArmConstants.kGearRatio);
    // Velocity factor: RPM -> mechanism radians/second ((2π / gearRatio) / 60)
    configuration.encoder.velocityConversionFactor(((2.0 * Math.PI) / ArmConstants.kGearRatio) / 60.0);

    m_motor.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (inputs == null) {
      return;
    }
    inputs.angleRadians = m_encoder.getPosition();
    inputs.velocityRadiansPerSecond = m_encoder.getVelocity();
    inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.currentAmps = m_motor.getOutputCurrent();

    // No physical limit switches wired to SPARK Flex digital inputs by default
    inputs.forwardLimitSwitchTripped = false;
    inputs.reverseLimitSwitchTripped = false;
  }

  @Override
  public void setVoltage(double appliedVolts) {
    if (!Double.isFinite(appliedVolts)) {
      m_motor.setVoltage(0.0);
      return;
    }
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_motor.setVoltage(clampedVolts);
  }

  @Override
  public void resetEncoder() {
    m_encoder.setPosition(0.0);
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    SparkFlexConfig updateConfig = new SparkFlexConfig();
    updateConfig.idleMode(enableBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    m_motor.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
