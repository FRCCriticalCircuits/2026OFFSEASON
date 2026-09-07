// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  class SwerveModuleIOInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSecond = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public Rotation2d steerAngle = new Rotation2d();
    public double steerVelocityRadiansPerSecond = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;
  }

  default void updateInputs(SwerveModuleIOInputs inputs) {}

  default void setDriveVoltage(double appliedVolts) {}

  default void setSteerAngle(Rotation2d steerAngle) {}

  default void setBrakeMode(boolean enableBrakeMode) {}
}
