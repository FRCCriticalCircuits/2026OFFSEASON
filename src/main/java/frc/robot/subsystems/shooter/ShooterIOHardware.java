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
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

/**
 * Complete hardware IO implementation for the expanded Shooter subsystem featuring:
 * <ul>
 *   <li><b>4x Kraken X60 (TalonFX) Flywheels</b>: 1 Leader + 3 Followers on CANivore/RIO CAN bus.</li>
 *   <li><b>1x Kraken X44/X60 (TalonFX) Adjustable Hood</b>: Precision position control.</li>
 *   <li><b>2x NEO Vortex (SPARK Flex) Accelerator / Kicker</b>: High-speed dual-roller feeding directly into the flywheel.</li>
 * </ul>
 */
public class ShooterIOHardware implements ShooterIO {
  // ── Flywheel Motors (4x Kraken X60) ─────────────────────────────────────────
  private final TalonFX m_flywheelLeader;
  private final TalonFX m_flywheelFollower1;
  private final TalonFX m_flywheelFollower2;
  private final TalonFX m_flywheelFollower3;

  private final VelocityVoltage m_flywheelVelocityControl = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut m_flywheelVoltageControl = new VoltageOut(0.0).withEnableFOC(true);

  // ── Hood Motor (1x Kraken) ──────────────────────────────────────────────────
  private final TalonFX m_hoodMotor;
  private final PositionVoltage m_hoodPositionControl = new PositionVoltage(0.0).withEnableFOC(true);
  private final VoltageOut m_hoodVoltageControl = new VoltageOut(0.0).withEnableFOC(true);

  // ── Accelerator Motors (2x NEO Vortex on SPARK Flex) ────────────────────────
  private final SparkFlex m_acceleratorLeader;
  private final SparkFlex m_acceleratorFollower;
  private final RelativeEncoder m_acceleratorEncoder;

  private double m_flywheelTargetVelocityRotationsPerSecond = 0.0;
  private double m_acceleratorTargetVelocityRotationsPerSecond = 0.0;
  private double m_hoodTargetAngleRadians = 0.0;

  /**
   * Constructs a ShooterIOHardware instance with all 7 configured motors.
   *
   * @param flywheelLeaderCanId CAN ID of the primary flywheel Kraken X60.
   * @param flywheelFollower1CanId CAN ID of the 1st follower flywheel Kraken X60.
   * @param flywheelFollower2CanId CAN ID of the 2nd follower flywheel Kraken X60.
   * @param flywheelFollower3CanId CAN ID of the 3rd follower flywheel Kraken X60.
   * @param hoodMotorCanId CAN ID of the adjustable hood Kraken motor.
   * @param acceleratorLeaderCanId CAN ID of the leader accelerator SPARK Flex.
   * @param acceleratorFollowerCanId CAN ID of the follower accelerator SPARK Flex.
   */
  public ShooterIOHardware(
      int flywheelLeaderCanId,
      int flywheelFollower1CanId,
      int flywheelFollower2CanId,
      int flywheelFollower3CanId,
      int hoodMotorCanId,
      int acceleratorLeaderCanId,
      int acceleratorFollowerCanId) {

    CANBus canbus = new CANBus(Constants.kCANBusName);

    // ── 1. Flywheel 4-Kraken Array ──────────────────────────────────────────
    m_flywheelLeader = new TalonFX(flywheelLeaderCanId, canbus);
    m_flywheelFollower1 = new TalonFX(flywheelFollower1CanId, canbus);
    m_flywheelFollower2 = new TalonFX(flywheelFollower2CanId, canbus);
    m_flywheelFollower3 = new TalonFX(flywheelFollower3CanId, canbus);

    TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
    flywheelLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    flywheelLeaderConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.kFlywheelStatorCurrentLimitAmps;
    flywheelLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.kFlywheelSupplyCurrentLimitAmps;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelLeaderConfig.Slot0.kP = ShooterConstants.kFlywheelProportionalGain;
    flywheelLeaderConfig.Slot0.kI = ShooterConstants.kFlywheelIntegralGain;
    flywheelLeaderConfig.Slot0.kD = ShooterConstants.kFlywheelDerivativeGain;
    flywheelLeaderConfig.Slot0.kS = ShooterConstants.kFlywheelStaticGain;
    flywheelLeaderConfig.Slot0.kV = ShooterConstants.kFlywheelVelocityGain;
    flywheelLeaderConfig.Slot0.kA = ShooterConstants.kFlywheelAccelerationGain;

    m_flywheelLeader.getConfigurator().apply(flywheelLeaderConfig);

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.kFlywheelStatorCurrentLimitAmps;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followerConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.kFlywheelSupplyCurrentLimitAmps;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_flywheelFollower1.getConfigurator().apply(followerConfig);
    m_flywheelFollower1.setControl(
        new Follower(flywheelLeaderCanId, MotorAlignmentValue.Opposed));

    m_flywheelFollower2.getConfigurator().apply(followerConfig);
    m_flywheelFollower2.setControl(
        new Follower(flywheelLeaderCanId, MotorAlignmentValue.Aligned));

    m_flywheelFollower3.getConfigurator().apply(followerConfig);
    m_flywheelFollower3.setControl(
        new Follower(flywheelLeaderCanId, MotorAlignmentValue.Opposed));

    // ── 2. Adjustable Hood ───────────────────────────────────────────────────
    m_hoodMotor = new TalonFX(hoodMotorCanId, canbus);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kHoodStatorCurrentLimitAmps;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kHoodSupplyCurrentLimitAmps;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodConfig.Slot0.kP = ShooterConstants.kHoodProportionalGain;
    hoodConfig.Slot0.kI = ShooterConstants.kHoodIntegralGain;
    hoodConfig.Slot0.kD = ShooterConstants.kHoodDerivativeGain;

    m_hoodMotor.getConfigurator().apply(hoodConfig);

    // ── 3. Accelerator / Kicker (2x NEO Vortex on SPARK Flex) ───────────────
    m_acceleratorLeader = new SparkFlex(acceleratorLeaderCanId, MotorType.kBrushless);
    m_acceleratorFollower = new SparkFlex(acceleratorFollowerCanId, MotorType.kBrushless);
    m_acceleratorEncoder = m_acceleratorLeader.getEncoder();

    SparkFlexConfig accelLeaderConfig = new SparkFlexConfig();
    accelLeaderConfig.idleMode(IdleMode.kCoast);
    accelLeaderConfig.smartCurrentLimit(ShooterConstants.kAcceleratorSmartCurrentLimitAmps);
    accelLeaderConfig.voltageCompensation(ShooterConstants.kAcceleratorVoltageCompensationVolts);
    m_acceleratorLeader.configure(
        accelLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig accelFollowerConfig = new SparkFlexConfig();
    accelFollowerConfig.idleMode(IdleMode.kCoast);
    accelFollowerConfig.smartCurrentLimit(ShooterConstants.kAcceleratorSmartCurrentLimitAmps);
    accelFollowerConfig.voltageCompensation(ShooterConstants.kAcceleratorVoltageCompensationVolts);
    accelFollowerConfig.follow(acceleratorLeaderCanId, ShooterConstants.kAcceleratorFollowerInverted);
    m_acceleratorFollower.configure(
        accelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Convenience constructor using defaults from ShooterConstants. */
  public ShooterIOHardware() {
    this(
        ShooterConstants.kFlywheelLeaderMotorId,
        ShooterConstants.kFlywheelFollower1MotorId,
        ShooterConstants.kFlywheelFollower2MotorId,
        ShooterConstants.kFlywheelFollower3MotorId,
        ShooterConstants.kHoodMotorId,
        ShooterConstants.kAcceleratorLeaderMotorId,
        ShooterConstants.kAcceleratorFollowerMotorId);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (inputs == null) {
      return;
    }
    // Flywheel inputs
    inputs.flywheelVelocityRotationsPerSecond = m_flywheelLeader.getVelocity().getValueAsDouble();
    inputs.flywheelTargetVelocityRotationsPerSecond = m_flywheelTargetVelocityRotationsPerSecond;
    inputs.flywheelAppliedVolts = m_flywheelLeader.getMotorVoltage().getValueAsDouble();
    inputs.flywheelLeaderCurrentAmps = m_flywheelLeader.getStatorCurrent().getValueAsDouble();
    inputs.flywheelFollower1CurrentAmps = m_flywheelFollower1.getStatorCurrent().getValueAsDouble();
    inputs.flywheelFollowerCurrentAmps = inputs.flywheelFollower1CurrentAmps;
    inputs.flywheelFollower2CurrentAmps = m_flywheelFollower2.getStatorCurrent().getValueAsDouble();
    inputs.flywheelFollower3CurrentAmps = m_flywheelFollower3.getStatorCurrent().getValueAsDouble();

    // Accelerator inputs
    inputs.acceleratorVelocityRotationsPerSecond = m_acceleratorEncoder.getVelocity() / 60.0;
    inputs.acceleratorTargetVelocityRotationsPerSecond = m_acceleratorTargetVelocityRotationsPerSecond;
    inputs.acceleratorAppliedVolts =
        m_acceleratorLeader.getAppliedOutput() * m_acceleratorLeader.getBusVoltage();
    inputs.acceleratorLeaderCurrentAmps = m_acceleratorLeader.getOutputCurrent();
    inputs.acceleratorFollowerCurrentAmps = m_acceleratorFollower.getOutputCurrent();

    // Hood inputs
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
    m_flywheelLeader.setControl(
        m_flywheelVelocityControl.withVelocity(velocityRotationsPerSecond));
  }

  @Override
  public void setFlywheelVoltage(double appliedVolts) {
    m_flywheelTargetVelocityRotationsPerSecond = 0.0;
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_flywheelLeader.setControl(m_flywheelVoltageControl.withOutput(clampedVolts));
  }

  @Override
  public void stopFlywheel() {
    m_flywheelTargetVelocityRotationsPerSecond = 0.0;
    m_flywheelLeader.stopMotor();
  }

  // ── Accelerator Control ────────────────────────────────────────────────────

  @Override
  public void setAcceleratorVelocity(double velocityRotationsPerSecond) {
    m_acceleratorTargetVelocityRotationsPerSecond = velocityRotationsPerSecond;
    if (Math.abs(velocityRotationsPerSecond) < 1e-4) {
      stopAccelerator();
      return;
    }
    double feedforwardVolts =
        velocityRotationsPerSecond * ShooterConstants.kAcceleratorVelocityGain;
    setAcceleratorVoltage(feedforwardVolts);
  }

  @Override
  public void setAcceleratorVoltage(double appliedVolts) {
    if (!Double.isFinite(appliedVolts)) {
      m_acceleratorLeader.setVoltage(0.0);
      return;
    }
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_acceleratorLeader.setVoltage(clampedVolts);
  }

  @Override
  public void stopAccelerator() {
    m_acceleratorTargetVelocityRotationsPerSecond = 0.0;
    m_acceleratorLeader.stopMotor();
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
