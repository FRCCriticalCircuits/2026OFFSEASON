// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

/**
 * Hardware IO implementation for the Shooter subsystem using Kraken X60 (TalonFX) motors on CANivore
 * for both the dual flywheel and adjustable hood.
 */
public class ShooterIOKraken implements ShooterIO {
  // ── Flywheel Motors ────────────────────────────────────────────────────────
  private final TalonFX m_flywheelLeaderMotor;
  private final TalonFX m_flywheelFollowerMotor;

  private final VelocityVoltage m_flywheelVelocityControl = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut m_flywheelVoltageControl = new VoltageOut(0.0).withEnableFOC(true);

  // ── Hood Motor ─────────────────────────────────────────────────────────────
  private final TalonFX m_hoodMotor;
  private final PositionVoltage m_hoodPositionControl = new PositionVoltage(0.0).withEnableFOC(true);
  private final VoltageOut m_hoodVoltageControl = new VoltageOut(0.0).withEnableFOC(true);

  private double m_flywheelTargetVelocityRotationsPerSecond = 0.0;
  private double m_hoodTargetAngleRadians = 0.0;

  /**
   * Constructs a ShooterIOKraken instance.
   *
   * @param flywheelLeaderCanId CAN ID of the primary flywheel TalonFX motor.
   * @param flywheelFollowerCanId CAN ID of the secondary flywheel TalonFX motor.
   * @param hoodMotorCanId CAN ID of the adjustable hood TalonFX motor.
   */
  public ShooterIOKraken(int flywheelLeaderCanId, int flywheelFollowerCanId, int hoodMotorCanId) {
    CANBus canbus = new CANBus(Constants.kCANBusName);

    // ── 1. Flywheel Configuration ───────────────────────────────────────────
    m_flywheelLeaderMotor = new TalonFX(flywheelLeaderCanId, canbus);
    m_flywheelFollowerMotor = new TalonFX(flywheelFollowerCanId, canbus);

    TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
    flywheelLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Standard Kraken X60 current limits (Stator: 80A rapid spin-up, Supply: 60A high-power flywheel limit)
    flywheelLeaderConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.kFlywheelStatorCurrentLimitAmps;
    flywheelLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.kFlywheelSupplyCurrentLimitAmps;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Slot 0 velocity PID & feedforward gains
    flywheelLeaderConfig.Slot0.kP = ShooterConstants.kFlywheelProportionalGain;
    flywheelLeaderConfig.Slot0.kI = ShooterConstants.kFlywheelIntegralGain;
    flywheelLeaderConfig.Slot0.kD = ShooterConstants.kFlywheelDerivativeGain;
    flywheelLeaderConfig.Slot0.kS = ShooterConstants.kFlywheelStaticGain;
    flywheelLeaderConfig.Slot0.kV = ShooterConstants.kFlywheelVelocityGain;
    flywheelLeaderConfig.Slot0.kA = ShooterConstants.kFlywheelAccelerationGain;

    m_flywheelLeaderMotor.getConfigurator().apply(flywheelLeaderConfig);

    TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
    flywheelFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.kFlywheelStatorCurrentLimitAmps;
    flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.kFlywheelSupplyCurrentLimitAmps;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerConfig);
    m_flywheelFollowerMotor.setControl(
        new Follower(flywheelLeaderCanId, MotorAlignmentValue.Opposed));

    // ── 2. Hood Configuration ───────────────────────────────────────────────
    m_hoodMotor = new TalonFX(hoodMotorCanId, canbus);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Standard Kraken X44 / X60 current limits (Stator: 40A position holding, Supply: 40A breaker protection)
    hoodConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.kHoodStatorCurrentLimitAmps;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.kHoodSupplyCurrentLimitAmps;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Slot 0 position PID gains for hood
    hoodConfig.Slot0.kP = ShooterConstants.kHoodProportionalGain;
    hoodConfig.Slot0.kI = ShooterConstants.kHoodIntegralGain;
    hoodConfig.Slot0.kD = ShooterConstants.kHoodDerivativeGain;

    m_hoodMotor.getConfigurator().apply(hoodConfig);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Flywheel inputs
    inputs.flywheelVelocityRotationsPerSecond = m_flywheelLeaderMotor.getVelocity().getValueAsDouble();
    inputs.flywheelTargetVelocityRotationsPerSecond = m_flywheelTargetVelocityRotationsPerSecond;
    inputs.flywheelAppliedVolts = m_flywheelLeaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.flywheelLeaderCurrentAmps = m_flywheelLeaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.flywheelFollowerCurrentAmps = m_flywheelFollowerMotor.getStatorCurrent().getValueAsDouble();

    // Hood inputs: convert motor rotations -> mechanism radians
    double motorRotations = m_hoodMotor.getPosition().getValueAsDouble();
    inputs.hoodAngleRadians = (motorRotations / ShooterConstants.kHoodGearRatio) * (2.0 * Math.PI);
    inputs.hoodTargetAngleRadians = m_hoodTargetAngleRadians;
    inputs.hoodAppliedVolts = m_hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.hoodCurrentAmps = m_hoodMotor.getStatorCurrent().getValueAsDouble();
  }

  // ── Flywheel Control ───────────────────────────────────────────────────────

  @Override
  public void setFlywheelVelocity(double velocityRotationsPerSecond) {
    m_flywheelTargetVelocityRotationsPerSecond = velocityRotationsPerSecond;
    m_flywheelLeaderMotor.setControl(
        m_flywheelVelocityControl.withVelocity(velocityRotationsPerSecond));
  }

  @Override
  public void setFlywheelVoltage(double appliedVolts) {
    m_flywheelTargetVelocityRotationsPerSecond = 0.0;
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_flywheelLeaderMotor.setControl(m_flywheelVoltageControl.withOutput(clampedVolts));
  }

  @Override
  public void stopFlywheel() {
    m_flywheelTargetVelocityRotationsPerSecond = 0.0;
    m_flywheelLeaderMotor.stopMotor();
  }

  // ── Hood Control ───────────────────────────────────────────────────────────

  @Override
  public void setHoodAngle(double angleRadians) {
    m_hoodTargetAngleRadians =
        MathUtil.clamp(
            angleRadians,
            ShooterConstants.kHoodMinAngleRadians,
            ShooterConstants.kHoodMaxAngleRadians);
    double motorRotations =
        (m_hoodTargetAngleRadians / (2.0 * Math.PI)) * ShooterConstants.kHoodGearRatio;
    m_hoodMotor.setControl(m_hoodPositionControl.withPosition(motorRotations));
  }

  @Override
  public void setHoodVoltage(double appliedVolts) {
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_hoodMotor.setControl(m_hoodVoltageControl.withOutput(clampedVolts));
  }

  @Override
  public void stopHood() {
    m_hoodMotor.stopMotor();
  }

  @Override
  public void resetHoodEncoder() {
    m_hoodMotor.setPosition(0.0);
  }
}
