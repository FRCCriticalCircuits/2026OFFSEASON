// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 m_pigeon;

  public GyroIOPigeon2(int pigeonCanId) {
    m_pigeon = new Pigeon2(pigeonCanId, new CANBus(Constants.kCANBusName));
  }

  @Override
  public void updateInputs(GyroIOInputs gyroInputs) {
    var yawSignal = m_pigeon.getYaw();
    gyroInputs.yawAngle = Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
    gyroInputs.yawRateDegreesPerSecond = m_pigeon.getAngularVelocityZWorld().getValueAsDouble();
    gyroInputs.isConnected = BaseStatusSignal.isAllGood(yawSignal);
  }
}
