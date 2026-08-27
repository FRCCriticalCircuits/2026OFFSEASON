// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
  private Rotation2d m_yawAngle = new Rotation2d();

  @Override
  public void updateInputs(GyroIOInputs gyroInputs) {
    gyroInputs.isConnected = true;
    gyroInputs.yawAngle = m_yawAngle;
    gyroInputs.yawRateDegreesPerSecond = 0.0;
  }

  public void setYawAngle(Rotation2d yawAngle) {
    m_yawAngle = yawAngle;
  }
}
