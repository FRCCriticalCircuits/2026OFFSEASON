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
 *   <li><b>4x Kraken X60 (TalonFX) Flywheels</b>: 1 Leader + 3 Followers on CANivore/RIO CAN bus.</li>
 *   <li><b>1x Kraken X60 (TalonFX) Adjustable Hood</b>: CAN ID 37 with closed-loop PositionVoltage.</li>
 *   <li><b>1x REV NEO Vortex (SPARK MAX) Supporting Shooter</b>: CAN ID 42 kicker/supporting roller.</li>
 * </ul>
 */
public class ShooterIOHardware implements ShooterIO, AutoCloseable {
  // ── Flywheel Motors (4x Kraken X60 on TalonFX) ──────────────────────────────
  private final TalonFX m_flywheelLeader;
  private final TalonFX m_flywheelFollower1;
  private final TalonFX m_flywheelFollower2;
  private final TalonFX m_flywheelFollower3;

  private final VelocityVoltage m_flywheelVelocityControl = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut m_flywheelVoltageControl = new VoltageOut(0.0).withEnableFOC(true);

  // ── Hood Motor (1x Kraken X60 on TalonFX) ───────────────────────────────────
  private final TalonFX m_hoodMotor;
  private final PositionVoltage m_hoodPositionControl = new PositionVoltage(0.0).withEnableFOC(true);
  private final VoltageOut m_hoodVoltageControl = new VoltageOut(0.0).withEnableFOC(true);

  // ── Supporting Shooter (1x NEO Vortex on SPARK MAX) ─────────────────────────
  private final SparkMax m_supportingShooterMotor;
  private final RelativeEncoder m_supportingShooterEncoder;

  private double m_flywheelTargetVelocityRotationsPerSecond = 0.0;
  private double m_hoodTargetAngleRadians = 0.0;
  private double m_supportingShooterTargetVelocityRotationsPerSecond = 0.0;

  /**
   * Constructs a ShooterIOHardware instance with all 6 configured motors.
   *
   * @param flywheelLeaderCanId CAN ID of the primary flywheel Kraken X60.
   * @param flywheelFollower1CanId CAN ID of the 1st follower flywheel Kraken X60.
   * @param flywheelFollower2CanId CAN ID of the 2nd follower flywheel Kraken X60.
   * @param flywheelFollower3CanId CAN ID of the 3rd follower flywheel Kraken X60.
   * @param hoodMotorCanId CAN ID of the adjustable hood Kraken X60 (TalonFX).
   * @param supportingShooterCanId CAN ID of the supporting shooter NEO Vortex (SPARK MAX).
   */
  public ShooterIOHardware(
      int flywheelLeaderCanId,
      int flywheelFollower1CanId,
      int flywheelFollower2CanId,
      int flywheelFollower3CanId,
      int hoodMotorCanId,
      int supportingShooterCanId) {

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

    // Follower 1 (CAN ID 36) follows Leader 35 (Aligned)
    m_flywheelFollower1.getConfigurator().apply(followerConfig);
    m_flywheelFollower1.setControl(
        new Follower(flywheelLeaderCanId, MotorAlignmentValue.Aligned));

    // Follower 2 (CAN ID 38) opposed to Leader 35
    m_flywheelFollower2.getConfigurator().apply(followerConfig);
    m_flywheelFollower2.setControl(
        new Follower(flywheelLeaderCanId, MotorAlignmentValue.Opposed));

    // Follower 3 (CAN ID 39) opposed to Leader 35
    m_flywheelFollower3.getConfigurator().apply(followerConfig);
    m_flywheelFollower3.setControl(
        new Follower(flywheelLeaderCanId, MotorAlignmentValue.Opposed));

    // ── 2. Adjustable Hood (Kraken X60 on TalonFX) ───────────────────────────
    m_hoodMotor = new TalonFX(hoodMotorCanId, canbus);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kHoodStatorCurrentLimitAmps;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kHoodSupplyCurrentLimitAmps;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodConfig.Slot0.kP = ShooterConstants.kHoodProportionalGain;
    hoodConfig.Slot0.kI = ShooterConstants.kHoodIntegralGain;
    hoodConfig.Slot0.kD = ShooterConstants.kHoodDerivativeGain;
    hoodConfig.Slot0.kS = ShooterConstants.kHoodStaticGain;

    // 50ms ramp period for smooth hood acceleration/deceleration
    hoodConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

    m_hoodMotor.getConfigurator().apply(hoodConfig);

    // ── 3. Supporting Shooter (1x NEO Vortex on SPARK MAX CAN ID 42) ─────────
    m_supportingShooterMotor = new SparkMax(supportingShooterCanId, MotorType.kBrushless);
    m_supportingShooterEncoder = m_supportingShooterMotor.getEncoder();

    SparkMaxConfig supportingConfig = new SparkMaxConfig();
    supportingConfig.inverted(false);
    supportingConfig.idleMode(IdleMode.kCoast);
    supportingConfig.smartCurrentLimit(ShooterConstants.kSupportingShooterSmartCurrentLimitAmps);
    supportingConfig.voltageCompensation(ShooterConstants.kSupportingShooterVoltageCompensationVolts);

    m_supportingShooterMotor.configure(
        supportingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Convenience constructor using defaults from ShooterConstants. */
  public ShooterIOHardware() {
    this(
        ShooterConstants.kFlywheelLeaderMotorId,
        ShooterConstants.kFlywheelFollower1MotorId,
        ShooterConstants.kFlywheelFollower2MotorId,
        ShooterConstants.kFlywheelFollower3MotorId,
        ShooterConstants.kHoodMotorId,
        ShooterConstants.kSupportingShooterMotorId);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (inputs == null) {
      return;
    }
    // Flywheel inputs (4 Krakens)
    inputs.flywheelVelocityRotationsPerSecond = m_flywheelLeader.getVelocity().getValueAsDouble();
    inputs.flywheelTargetVelocityRotationsPerSecond = m_flywheelTargetVelocityRotationsPerSecond;
    inputs.flywheelAppliedVolts = m_flywheelLeader.getMotorVoltage().getValueAsDouble();
    inputs.flywheelLeaderCurrentAmps = m_flywheelLeader.getStatorCurrent().getValueAsDouble();
    inputs.flywheelFollower1CurrentAmps = m_flywheelFollower1.getStatorCurrent().getValueAsDouble();
    inputs.flywheelFollowerCurrentAmps = inputs.flywheelFollower1CurrentAmps;
    inputs.flywheelFollower2CurrentAmps = m_flywheelFollower2.getStatorCurrent().getValueAsDouble();
    inputs.flywheelFollower3CurrentAmps = m_flywheelFollower3.getStatorCurrent().getValueAsDouble();

    // Hood inputs (Kraken X60)
    double direction = ShooterConstants.kHoodInverted ? -1.0 : 1.0;
    double motorRotations = m_hoodMotor.getPosition().getValueAsDouble();
    inputs.hoodAngleRadians = direction * (motorRotations / ShooterConstants.kHoodGearRatio) * (2.0 * Math.PI);
    inputs.hoodTargetAngleRadians = m_hoodTargetAngleRadians;
    inputs.hoodAppliedVolts = m_hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.hoodCurrentAmps = m_hoodMotor.getStatorCurrent().getValueAsDouble();

    // Supporting Shooter inputs (NEO Vortex)
    inputs.supportingShooterVelocityRotationsPerSecond = m_supportingShooterEncoder.getVelocity() / 60.0;
    inputs.supportingShooterTargetVelocityRotationsPerSecond = m_supportingShooterTargetVelocityRotationsPerSecond;
    inputs.supportingShooterAppliedVolts =
        m_supportingShooterMotor.getAppliedOutput() * m_supportingShooterMotor.getBusVoltage();
    inputs.supportingShooterCurrentAmps = m_supportingShooterMotor.getOutputCurrent();
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

  // ── Hood Control (Kraken X60 PositionVoltage) ──────────────────────────────

  @Override
  public void setHoodAngle(double angleRadians) {
    m_hoodTargetAngleRadians =
        MathUtil.clamp(
            angleRadians,
            ShooterConstants.kHoodMinAngleRadians,
            ShooterConstants.kHoodMaxAngleRadians);
    double direction = ShooterConstants.kHoodInverted ? -1.0 : 1.0;
    double motorRotations =
        direction * (m_hoodTargetAngleRadians / (2.0 * Math.PI)) * ShooterConstants.kHoodGearRatio;
    m_hoodMotor.setControl(m_hoodPositionControl.withPosition(motorRotations));
  }

  @Override
  public void setHoodVoltage(double appliedVolts) {
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    double direction = ShooterConstants.kHoodInverted ? -1.0 : 1.0;
    m_hoodMotor.setControl(m_hoodVoltageControl.withOutput(direction * clampedVolts));
  }

  @Override
  public void stopHood() {
    m_hoodMotor.stopMotor();
  }

  @Override
  public void resetHoodEncoder() {
    m_hoodMotor.setPosition(0.0);
  }

  // ── Supporting Shooter Control (NEO Vortex SPARK MAX) ───────────────────────

  @Override
  public void setSupportingShooterVelocity(double velocityRotationsPerSecond) {
    m_supportingShooterTargetVelocityRotationsPerSecond = velocityRotationsPerSecond;
    if (Math.abs(velocityRotationsPerSecond) < 1e-4) {
      stopSupportingShooter();
      return;
    }
    double feedforwardVolts =
        velocityRotationsPerSecond * ShooterConstants.kSupportingShooterVelocityGain
            + Math.signum(velocityRotationsPerSecond) * ShooterConstants.kSupportingShooterStaticGain;
    setSupportingShooterVoltage(feedforwardVolts);
  }

  @Override
  public void setSupportingShooterVoltage(double appliedVolts) {
    if (!Double.isFinite(appliedVolts)) {
      m_supportingShooterMotor.stopMotor();
      return;
    }
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_supportingShooterMotor.setVoltage(clampedVolts);
  }

  @Override
  public void stopSupportingShooter() {
    m_supportingShooterTargetVelocityRotationsPerSecond = 0.0;
    m_supportingShooterMotor.stopMotor();
  }

  @Override
  public void close() {
    m_supportingShooterMotor.close();
  }
}
