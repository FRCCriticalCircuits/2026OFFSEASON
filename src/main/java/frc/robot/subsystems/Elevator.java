// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  // TODO: Replace with real motor controller (e.g. TalonFX, SparkMax)
  // private final TalonFX m_motor = new TalonFX(ElevatorConstants.kMotorId);

  // Feedforward constants — tune these
  private static final double kS = 0.0;
  private static final double kG = 0.0; // gravity compensation (volts)
  private static final double kV = 0.0;
  private static final double kA = 0.0;

  // PID constants — tune these
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Motion profile constraints (meters/s, meters/s^2) — tune these
  private static final double kMaxVelocity     = 1.0;
  private static final double kMaxAcceleration = 2.0;

  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(kS, kG, kV, kA);

  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          kP, kI, kD,
          new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));

  /** Height tolerance in meters considered "at goal". */
  private static final double kTolerance = 0.01;

  private double m_goalMeters = 0.0;

  public Elevator() {
    m_controller.setTolerance(kTolerance);
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Sets the desired height goal for the elevator.
   *
   * @param meters target height in meters
   */
  public void setGoal(double meters) {
    m_goalMeters = meters;
    m_controller.setGoal(meters);
  }

  /** @return true when the elevator is at the goal height within tolerance */
  public boolean atGoal() {
    return m_controller.atGoal();
  }

  /** @return current elevator height in meters */
  public double getHeightMeters() {
    // TODO: return real encoder position, e.g.:
    // return m_motor.getPosition().getValueAsDouble() * kMetersPerRotation;
    return 0.0;
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    double currentHeight = getHeightMeters();
    double pidOutput    = m_controller.calculate(currentHeight);
    double ffOutput     = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    double voltage      = pidOutput + ffOutput;

    // TODO: Apply voltage to real motor, e.g.:
    // m_motor.setVoltage(voltage);

    SmartDashboard.putNumber("Elevator/Height (m)",    currentHeight);
    SmartDashboard.putNumber("Elevator/Goal (m)",      m_goalMeters);
    SmartDashboard.putNumber("Elevator/Output (V)",    voltage);
    SmartDashboard.putBoolean("Elevator/At Goal",      atGoal());
  }
}
