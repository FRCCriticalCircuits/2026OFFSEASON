// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

/**
 * Hardware IO implementation for the arm subsystem using a Kraken X60 (TalonFX) motor.
 */
public class ArmIOKraken implements ArmIO {
  private final TalonFX m_motor;
  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  /**
   * Constructs an ArmIOKraken instance.
   *
   * @param motorCanId CAN ID of the arm TalonFX motor.
   */
  public ArmIOKraken(int motorCanId) {
    m_motor = new TalonFX(motorCanId, new CANBus(Constants.kCANBusName));

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Standard Kraken X60 current limits (Stator: 60A peak torque limit, Supply: 40A breaker protection)
    configuration.CurrentLimits.StatorCurrentLimit = ArmConstants.kStatorCurrentLimitAmps;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = ArmConstants.kSupplyCurrentLimitAmps;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_motor.getConfigurator().apply(configuration);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Read position: rotations -> radians (position / gearRatio * 2π)
    double positionRotations = m_motor.getPosition().getValueAsDouble();
    inputs.angleRadians = (positionRotations / ArmConstants.kGearRatio) * 2.0 * Math.PI;

    // Read velocity: rotations/s -> rad/s (velocity / gearRatio * 2π)
    double velocityRotationsPerSecond = m_motor.getVelocity().getValueAsDouble();
    inputs.velocityRadiansPerSecond =
        (velocityRotationsPerSecond / ArmConstants.kGearRatio) * 2.0 * Math.PI;

    // Applied voltage and stator current
    inputs.appliedVolts = m_motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = m_motor.getStatorCurrent().getValueAsDouble();

    // Limit switches: false (no physical limit switches on TalonFX by default)
    inputs.forwardLimitSwitchTripped = false;
    inputs.reverseLimitSwitchTripped = false;
  }

  @Override
  public void setVoltage(double appliedVolts) {
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_motor.setControl(m_voltageOut.withOutput(clampedVolts));
  }

  @Override
  public void resetEncoder() {
    m_motor.setPosition(0.0);
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    m_motor.setNeutralMode(enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
