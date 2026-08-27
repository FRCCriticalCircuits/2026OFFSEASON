// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOKraken implements SwerveModuleIO {
  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_cancoder;

  private final VoltageOut m_driveVoltageControl = new VoltageOut(0.0).withEnableFOC(true);
  private final PositionVoltage m_steerPositionControl = new PositionVoltage(0.0).withEnableFOC(true);

  public SwerveModuleIOKraken(
      int driveMotorCanId,
      int steerMotorCanId,
      int cancoderCanId,
      double cancoderOffsetRotations,
      boolean isDriveInverted) {
    CANBus canbus = new CANBus(Constants.kCANBusName);

    m_driveMotor = new TalonFX(driveMotorCanId, canbus);
    m_steerMotor = new TalonFX(steerMotorCanId, canbus);
    m_cancoder = new CANcoder(cancoderCanId, canbus);

    CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
    cancoderConfiguration.MagnetSensor.MagnetOffset = cancoderOffsetRotations;
    m_cancoder.getConfigurator().apply(cancoderConfiguration);

    TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    driveConfiguration.MotorOutput.Inverted =
        isDriveInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfiguration.CurrentLimits.StatorCurrentLimit =
        SwerveConstants.kDriveStatorCurrentLimitAmps; // TODO: Tune this value
    driveConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfiguration.CurrentLimits.SupplyCurrentLimit =
        SwerveConstants.kDriveSupplyCurrentLimitAmps; // TODO: Tune this value
    driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_driveMotor.getConfigurator().apply(driveConfiguration);

    TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
    steerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    steerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfiguration.CurrentLimits.StatorCurrentLimit =
        SwerveConstants.kSteerStatorCurrentLimitAmps; // TODO: Tune this value
    steerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    steerConfiguration.CurrentLimits.SupplyCurrentLimit =
        SwerveConstants.kSteerSupplyCurrentLimitAmps; // TODO: Tune this value
    steerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    steerConfiguration.Slot0.kP = SwerveConstants.kSteerProportionalGain; // TODO: Tune this value
    steerConfiguration.Slot0.kI = SwerveConstants.kSteerIntegralGain; // TODO: Tune this value
    steerConfiguration.Slot0.kD = SwerveConstants.kSteerDerivativeGain; // TODO: Tune this value
    steerConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

    steerConfiguration.Feedback.FeedbackRemoteSensorID = cancoderCanId;
    steerConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    m_steerMotor.getConfigurator().apply(steerConfiguration);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionMeters =
        m_driveMotor.getPosition().getValueAsDouble()
            / SwerveConstants.kDriveGearRatio
            * 2.0
            * Math.PI
            * SwerveConstants.kWheelRadiusMeters;
    inputs.driveVelocityMetersPerSecond =
        m_driveMotor.getVelocity().getValueAsDouble()
            / SwerveConstants.kDriveGearRatio
            * 2.0
            * Math.PI
            * SwerveConstants.kWheelRadiusMeters;
    inputs.driveAppliedVolts = m_driveMotor.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = m_driveMotor.getStatorCurrent().getValueAsDouble();

    inputs.steerAngle =
        Rotation2d.fromRotations(m_cancoder.getAbsolutePosition().getValueAsDouble());
    inputs.steerVelocityRadiansPerSecond =
        m_cancoder.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
    inputs.steerAppliedVolts = m_steerMotor.getMotorVoltage().getValueAsDouble();
    inputs.steerCurrentAmps = m_steerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setDriveVoltage(double appliedVolts) {
    double clampedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
    m_driveMotor.setControl(m_driveVoltageControl.withOutput(clampedVolts));
  }

  @Override
  public void setSteerAngle(Rotation2d steerAngle) {
    m_steerMotor.setControl(m_steerPositionControl.withPosition(steerAngle.getRotations()));
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    NeutralModeValue mode = enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_driveMotor.setNeutralMode(mode);
    m_steerMotor.setNeutralMode(mode);
  }
}
