// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

/**
 * Complete hardware IO implementation for the Shooter subsystem featuring strictly 6 motors:
 * <ul>
 *   <li><b>5x Kraken X60 (TalonFX) Flywheels</b>: 1 Leader + 4 Followers on CANivore/RIO CAN bus.</li>
 *   <li><b>1x NEO Vortex (SPARK MAX) Adjustable Hood</b>: Precision position control.</li>
 * </ul>
 */
public class ShooterIOHardware implements ShooterIO, AutoCloseable {
  // ── Flywheel Motors (5x Kraken X60) ─────────────────────────────────────────
  private final TalonFX m_flywheelLeader;
  private final TalonFX m_flywheelFollower1;
  private final TalonFX m_flywheelFollower2;
  private final TalonFX m_flywheelFollower3;
  private final TalonFX m_flywheelFollower4;

  private final VelocityVoltage m_flywheelVelocityControl = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut m_flywheelVoltageControl = new VoltageOut(0.0).withEnableFOC(true);

  // ── Hood Motor (1x NEO Vortex on SPARK MAX) ─────────────────────────────────
  private final SparkMax m_hoodMotor;
  private final RelativeEncoder m_hoodEncoder;
  private final SparkClosedLoopController m_hoodClosedLoopController;

  private double m_flywheelTargetVelocityRotationsPerSecond = 0.0;
  private double m_hoodTargetAngleRadians = 0.0;

  /**
   * Constructs a ShooterIOHardware instance with all 6 configured motors.
   *
   * @param flywheelLeaderCanId CAN ID of the primary flywheel Kraken X60.
   * @param flywheelFollower1CanId CAN ID of the 1st follower flywheel Kraken X60.
   * @param flywheelFollower2CanId CAN ID of the 2nd follower flywheel Kraken X60.
   * @param flywheelFollower3CanId CAN ID of the 3rd follower flywheel Kraken X60.
   * @param flywheelFollower4CanId CAN ID of the 4th follower flywheel Kraken X60.
   * @param hoodMotorCanId CAN ID of the adjustable hood SPARK MAX controller.
   */
  public ShooterIOHardware(
      int flywheelLeaderCanId,
      int flywheelFollower1CanId,
      int flywheelFollower2CanId,
      int flywheelFollower3CanId,
      int flywheelFollower4CanId,
      int hoodMotorCanId) {

    CANBus canbus = new CANBus(Constants.kCANBusName);

    // ── 1. Flywheel 5-Kraken Array ──────────────────────────────────────────
    m_flywheelLeader = new TalonFX(flywheelLeaderCanId, canbus);
    m_flywheelFollower1 = new TalonFX(flywheelFollower1CanId, canbus);
    m_flywheelFollower2 = new TalonFX(flywheelFollower2CanId, canbus);
    m_flywheelFollower3 = new TalonFX(flywheelFollower3CanId, canbus);
    m_flywheelFollower4 = new TalonFX(flywheelFollower4CanId, canbus);

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

    m_flywheelFollower4.getConfigurator().apply(followerConfig);
    m_flywheelFollower4.setControl(
        new Follower(flywheelLeaderCanId, MotorAlignmentValue.Aligned));

    // ── 2. Adjustable Hood (NEO Vortex on SPARK MAX) ─────────────────────────
    m_hoodMotor = new SparkMax(hoodMotorCanId, MotorType.kBrushless);
    m_hoodEncoder = m_hoodMotor.getEncoder();
    m_hoodClosedLoopController = m_hoodMotor.getClosedLoopController();

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.smartCurrentLimit(ShooterConstants.kHoodSmartCurrentLimitAmps);
    hoodConfig.voltageCompensation(ShooterConstants.kHoodVoltageCompensationVolts);

    // Position factor: rotations -> mechanism radians (2π / gearRatio)
    hoodConfig.encoder.positionConversionFactor((2.0 * Math.PI) / ShooterConstants.kHoodGearRatio);
    // Velocity factor: RPM -> mechanism radians/second ((2π / gearRatio) / 60)
    hoodConfig.encoder.velocityConversionFactor(((2.0 * Math.PI) / ShooterConstants.kHoodGearRatio) / 60.0);

    hoodConfig.closedLoop.pid(
        ShooterConstants.kHoodProportionalGain,
        ShooterConstants.kHoodIntegralGain,
        ShooterConstants.kHoodDerivativeGain);

    m_hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Convenience constructor using defaults from ShooterConstants. */
  public ShooterIOHardware() {
    this(
        ShooterConstants.kFlywheelLeaderMotorId,
        ShooterConstants.kFlywheelFollower1MotorId,
        ShooterConstants.kFlywheelFollower2MotorId,
        ShooterConstants.kFlywheelFollower3MotorId,
        ShooterConstants.kFlywheelFollower4MotorId,
        ShooterConstants.kHoodMotorId);
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
    inputs.flywheelFollower4CurrentAmps = m_flywheelFollower4.getStatorCurrent().getValueAsDouble();

    // Hood inputs
    inputs.hoodAngleRadians = m_hoodEncoder.getPosition();
    inputs.hoodTargetAngleRadians = m_hoodTargetAngleRadians;
    inputs.hoodAppliedVolts = m_hoodMotor.getAppliedOutput() * m_hoodMotor.getBusVoltage();
    inputs.hoodCurrentAmps = m_hoodMotor.getOutputCurrent();
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

  // ── Hood Control ───────────────────────────────────────────────────────────

  @Override
  public void setHoodAngle(double angleRadians) {
    m_hoodTargetAngleRadians =
        MathUtil.clamp(
            angleRadians,
            ShooterConstants.kHoodMinAngleRadians,
            ShooterConstants.kHoodMaxAngleRadians);
    m_hoodClosedLoopController.setSetpoint(m_hoodTargetAngleRadians, ControlType.kPosition);
  }

  @Override
  public void setHoodVoltage(double appliedVolts) {
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_hoodMotor.setVoltage(clampedVolts);
  }

  @Override
  public void stopHood() {
    m_hoodMotor.stopMotor();
  }

  @Override
  public void resetHoodEncoder() {
    m_hoodEncoder.setPosition(0.0);
  }

  @Override
  public void close() {
    m_hoodMotor.close();
  }
}
