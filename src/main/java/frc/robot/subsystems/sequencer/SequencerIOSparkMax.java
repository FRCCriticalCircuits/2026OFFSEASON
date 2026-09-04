// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sequencer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.SequencerConstants;

/**
 * Hardware IO implementation for the Sequencer subsystem using dual NEO Vortex brushless motors
 * driven by SPARK MAX controllers in leader-follower configuration.
 */
public class SequencerIOSparkMax implements SequencerIO {
  private final SparkMax m_leaderMotor;
  private final SparkMax m_followerMotor;
  private final RelativeEncoder m_leaderEncoder;

  /**
   * Constructs a SequencerIOSparkMax instance with leader and follower CAN IDs.
   *
   * @param leaderCanId CAN ID of the primary/leader SPARK MAX controller.
   * @param followerCanId CAN ID of the secondary/follower SPARK MAX controller.
   */
  public SequencerIOSparkMax(int leaderCanId, int followerCanId) {
    m_leaderMotor = new SparkMax(leaderCanId, MotorType.kBrushless);
    m_followerMotor = new SparkMax(followerCanId, MotorType.kBrushless);
    m_leaderEncoder = m_leaderMotor.getEncoder();

    // Configure Leader
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.idleMode(IdleMode.kBrake);
    leaderConfig.smartCurrentLimit(SequencerConstants.kSmartCurrentLimitAmps);
    leaderConfig.voltageCompensation(SequencerConstants.kVoltageCompensationVolts);
    m_leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure Follower
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake);
    followerConfig.smartCurrentLimit(SequencerConstants.kSmartCurrentLimitAmps);
    followerConfig.voltageCompensation(SequencerConstants.kVoltageCompensationVolts);
    followerConfig.follow(leaderCanId, SequencerConstants.kFollowerInverted);
    m_followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(SequencerIOInputs inputs) {
    if (inputs == null) {
      return;
    }
    // Convert RPM -> RPS
    inputs.velocityRotationsPerSecond = m_leaderEncoder.getVelocity() / 60.0;
    inputs.positionRotations = m_leaderEncoder.getPosition();
    inputs.appliedVolts = m_leaderMotor.getAppliedOutput() * m_leaderMotor.getBusVoltage();
    inputs.leaderCurrentAmps = m_leaderMotor.getOutputCurrent();
    inputs.followerCurrentAmps = m_followerMotor.getOutputCurrent();
    inputs.currentAmps = inputs.leaderCurrentAmps + inputs.followerCurrentAmps;
  }

  @Override
  public void setVelocity(double velocityRotationsPerSecond) {
    if (Math.abs(velocityRotationsPerSecond) < 1e-4) {
      stop();
      return;
    }
    // Feedforward voltage calculation (kV + kS)
    double feedforwardVolts =
        velocityRotationsPerSecond * SequencerConstants.kVelocityGain
            + Math.signum(velocityRotationsPerSecond) * SequencerConstants.kStaticGain;
    setVoltage(feedforwardVolts);
  }

  @Override
  public void setVoltage(double appliedVolts) {
    if (!Double.isFinite(appliedVolts)) {
      m_leaderMotor.setVoltage(0.0);
      return;
    }
    double clampedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    m_leaderMotor.setVoltage(clampedVolts);
  }

  @Override
  public void stop() {
    m_leaderMotor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    IdleMode mode = enableBrakeMode ? IdleMode.kBrake : IdleMode.kCoast;

    SparkMaxConfig leaderUpdate = new SparkMaxConfig();
    leaderUpdate.idleMode(mode);
    m_leaderMotor.configure(leaderUpdate, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig followerUpdate = new SparkMaxConfig();
    followerUpdate.idleMode(mode);
    m_followerMotor.configure(followerUpdate, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void close() {
    m_leaderMotor.close();
    m_followerMotor.close();
  }
}
