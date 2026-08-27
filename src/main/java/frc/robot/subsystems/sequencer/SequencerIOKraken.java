// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sequencer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.SequencerConstants;

/**
 * Hardware IO implementation for the sequencer subsystem using a Kraken X60 (TalonFX) motor.
 */
public class SequencerIOKraken implements SequencerIO {
  private final TalonFX m_motor;
  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  /**
   * Constructs a SequencerIOKraken instance.
   *
   * @param motorCanId CAN ID of the sequencer TalonFX motor.
   */
  public SequencerIOKraken(int motorCanId) {
    m_motor = new TalonFX(motorCanId, new CANBus(Constants.kCANBusName));

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configuration.CurrentLimits.StatorCurrentLimit =
        SequencerConstants.kStatorCurrentLimitAmps; // TODO: Tune this value
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit =
        SequencerConstants.kSupplyCurrentLimitAmps; // TODO: Tune this value
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_motor.getConfigurator().apply(configuration);
  }

  @Override
  public void updateInputs(SequencerIOInputs inputs) {
    // Read position: rotations -> meters (position / gearRatio * 2π * drumRadius)
    double positionRotations = m_motor.getPosition().getValueAsDouble();
    inputs.heightMeters =
        (positionRotations / SequencerConstants.kGearRatio)
            * 2.0
            * Math.PI
            * SequencerConstants.kDrumRadiusMeters;

    // Read velocity: rotations/s -> m/s (velocity / gearRatio * 2π * drumRadius)
    double velocityRotationsPerSecond = m_motor.getVelocity().getValueAsDouble();
    inputs.velocityMetersPerSecond =
        (velocityRotationsPerSecond / SequencerConstants.kGearRatio)
            * 2.0
            * Math.PI
            * SequencerConstants.kDrumRadiusMeters;

    // Applied voltage and stator current
    inputs.appliedVolts = m_motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = m_motor.getStatorCurrent().getValueAsDouble();

    // Limit switches: false
    inputs.lowerLimitSwitchTripped = false;
    inputs.upperLimitSwitchTripped = false;
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
