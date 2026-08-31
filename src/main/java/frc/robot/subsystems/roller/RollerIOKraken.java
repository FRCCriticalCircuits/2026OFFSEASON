// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.RollerConstants;

/**
 * Hardware IO implementation for the Roller subsystem using a Kraken X60 (TalonFX) motor on CANivore.
 */
public class RollerIOKraken implements RollerIO {
  private final TalonFX m_motor;
  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  /**
   * Constructs a RollerIOKraken instance.
   *
   * @param motorCanId CAN ID of the roller TalonFX motor.
   */
  public RollerIOKraken(int motorCanId) {
    m_motor = new TalonFX(motorCanId, new CANBus(Constants.kCANBusName));

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Standard Kraken X60 current limits (Stator: 60A peak torque limit, Supply: 40A breaker protection)
    configuration.CurrentLimits.StatorCurrentLimit = RollerConstants.kStatorCurrentLimitAmps;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = RollerConstants.kSupplyCurrentLimitAmps;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_motor.getConfigurator().apply(configuration);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocityRotationsPerSecond = m_motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = m_motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = m_motor.getStatorCurrent().getValueAsDouble();
    inputs.gamePieceDetected = false; // Add digital / laserCAN / current spike check if installed
  }

  @Override
  public void setVoltage(double appliedVolts) {
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_motor.setControl(m_voltageOut.withOutput(clampedVolts));
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    m_motor.setNeutralMode(enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
