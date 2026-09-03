// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.RollerConstants;

/**
 * Hardware IO implementation for the Roller subsystem using dual Kraken X60 (TalonFX) motors on CANivore.
 */
public class RollerIOKraken implements RollerIO {
  private final TalonFX m_leaderMotor;
  private final TalonFX m_followerMotor;
  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  /**
   * Constructs a RollerIOKraken instance with leader and follower CAN IDs.
   *
   * @param leaderCanId CAN ID of the primary/leader roller TalonFX motor.
   * @param followerCanId CAN ID of the secondary/follower roller TalonFX motor.
   */
  public RollerIOKraken(int leaderCanId, int followerCanId) {
    CANBus canbus = new CANBus(Constants.kCANBusName);
    m_leaderMotor = new TalonFX(leaderCanId, canbus);
    m_followerMotor = new TalonFX(followerCanId, canbus);

    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Standard Kraken X60 current limits (Stator: 60A peak torque limit, Supply: 60A breaker protection)
    leaderConfig.CurrentLimits.StatorCurrentLimit = RollerConstants.kStatorCurrentLimitAmps;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.kSupplyCurrentLimitAmps;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_leaderMotor.getConfigurator().apply(leaderConfig);

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.CurrentLimits.StatorCurrentLimit = RollerConstants.kStatorCurrentLimitAmps;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followerConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.kSupplyCurrentLimitAmps;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_followerMotor.getConfigurator().apply(followerConfig);
    m_followerMotor.setControl(new Follower(leaderCanId, MotorAlignmentValue.Aligned));
  }

  /**
   * Constructs a RollerIOKraken instance with default follower CAN ID.
   *
   * @param leaderCanId CAN ID of the leader roller TalonFX motor.
   */
  public RollerIOKraken(int leaderCanId) {
    this(leaderCanId, RollerConstants.kFollowerMotorId);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (inputs == null) {
      return;
    }
    inputs.velocityRotationsPerSecond = m_leaderMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = m_leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.leaderCurrentAmps = m_leaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.followerCurrentAmps = m_followerMotor.getStatorCurrent().getValueAsDouble();
    inputs.currentAmps = inputs.leaderCurrentAmps;
    inputs.gamePieceDetected = false; // Add digital / laserCAN / current spike check if installed
  }

  @Override
  public void setVoltage(double appliedVolts) {
    if (!Double.isFinite(appliedVolts)) {
      m_leaderMotor.setControl(m_voltageOut.withOutput(0.0));
      return;
    }
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_leaderMotor.setControl(m_voltageOut.withOutput(clampedVolts));
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    NeutralModeValue mode = enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_leaderMotor.setNeutralMode(mode);
    m_followerMotor.setNeutralMode(mode);
  }
}
