// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
  class GyroIOInputs {
    public boolean isConnected = false;
    public Rotation2d yawAngle = new Rotation2d();
    public double yawRateDegreesPerSecond = 0.0;
  }

  default void updateInputs(GyroIOInputs gyroInputs) {}
}
