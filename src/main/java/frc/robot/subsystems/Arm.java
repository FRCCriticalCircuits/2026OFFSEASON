// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  // TODO: Replace with real motor controller (e.g. TalonFX, SparkMax)
  // private final TalonFX m_motor = new TalonFX(ArmConstants.kMotorId);

  // Feedforward constants — tune these
  private static final double kS = 0.0;
  private static final double kG = 0.0; // gravity compensation at horizontal (volts)
  private static final double kV = 0.0;
  private static final double kA = 0.0;

  // PID constants — tune these
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Motion profile constraints (rad/s, rad/s^2) — tune these
  private static final double kMaxVelocity     = Math.PI;
  private static final double kMaxAcceleration = Math.PI * 2.0;

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(kS, kG, kV, kA);

  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          kP, kI, kD,
          new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));

  /** Angle tolerance in radians considered "at goal". */
  private static final double kTolerance = Math.toRadians(2.0);

  private double m_goalRadians = 0.0;

  public Arm() {
    m_controller.setTolerance(kTolerance);
    // Enable continuous input if your arm can rotate past 2π
    // m_controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Sets the desired angle goal for the arm.
   *
   * @param radians target angle in radians (0 = horizontal)
   */
  public void setGoal(double radians) {
    m_goalRadians = radians;
    m_controller.setGoal(radians);
  }

  /** @return true when the arm is at the goal angle within tolerance */
  public boolean atGoal() {
    return m_controller.atGoal();
  }

  /** @return current arm angle in radians */
  public double getAngleRadians() {
    // TODO: return real encoder position, e.g.:
    // return m_encoder.getPosition() * kRadiansPerRotation;
    return 0.0;
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    double currentAngle = getAngleRadians();
    double pidOutput    = m_controller.calculate(currentAngle);
    double ffOutput     = m_feedforward.calculate(
        m_controller.getSetpoint().position,
        m_controller.getSetpoint().velocity);
    double voltage = pidOutput + ffOutput;

    // TODO: Apply voltage to real motor, e.g.:
    // m_motor.setVoltage(voltage);

    SmartDashboard.putNumber("Arm/Angle (deg)",   Math.toDegrees(currentAngle));
    SmartDashboard.putNumber("Arm/Goal (deg)",    Math.toDegrees(m_goalRadians));
    SmartDashboard.putNumber("Arm/Output (V)",    voltage);
    SmartDashboard.putBoolean("Arm/At Goal",      atGoal());
  }
}
