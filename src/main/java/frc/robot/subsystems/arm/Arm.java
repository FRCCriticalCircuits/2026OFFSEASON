// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import org.littletonrobotics.junction.Logger;

/**
 * Arm subsystem.
 *
 * <p>All hardware I/O is delegated to an {@link ArmIO} implementation injected
 * at construction time. The subsystem itself only contains control logic
 * (ProfiledPID + ArmFeedforward), making it trivially testable with any IO
 * backend (simulation, replay, or real hardware).
 *
 * <p>Typical instantiation in {@link frc.robot.RobotContainer}:
 * <pre>
 *   // Simulation / desktop testing
 *   Arm arm = new Arm(new ArmIOSim());
 *
 *   // Real robot hardware
 *   Arm arm = new Arm(new ArmIOKraken(ArmConstants.kMotorId));
 * </pre>
 */
public class Arm extends SubsystemBase {

  // ─── IO layer ─────────────────────────────────────────────────────────────

  private final ArmIO m_armIO;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();

  // ─── Controllers ──────────────────────────────────────────────────────────

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kStaticGain,
          ArmConstants.kGravityGain,
          ArmConstants.kVelocityGain,
          ArmConstants.kAccelerationGain);

  private final ProfiledPIDController m_feedbackController =
      new ProfiledPIDController(
          ArmConstants.kProportionalGain,
          ArmConstants.kIntegralGain,
          ArmConstants.kDerivativeGain,
          new TrapezoidProfile.Constraints(
              ArmConstants.kMaxVelocityRadiansPerSecond,
              ArmConstants.kMaxAccelerationRadiansPerSecondSquared));

  // ─── State ────────────────────────────────────────────────────────────────

  private double m_goalAngleRadians = 0.0;

  // ─── Constructor ──────────────────────────────────────────────────────────

  /**
   * Creates a new Arm subsystem backed by the provided IO implementation.
   *
   * @param armIO the hardware abstraction layer to use
   */
  public Arm(ArmIO armIO) {
    m_armIO = armIO;
    m_feedbackController.setTolerance(ArmConstants.kToleranceRadians);
    // Uncomment if your arm can rotate past 2π:
    // m_feedbackController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Sets the desired angle goal for the arm.
   *
   * @param targetAngleRadians target angle in radians (0 = horizontal)
   */
  public void setGoal(double targetAngleRadians) {
    m_goalAngleRadians = targetAngleRadians;
    m_feedbackController.setGoal(targetAngleRadians);
  }

  /** @return desired arm goal angle in radians */
  public double getGoalAngleRadians() {
    return m_goalAngleRadians;
  }

  /** @return true when the arm is at the goal angle within tolerance */
  public boolean atGoal() {
    return m_feedbackController.atGoal();
  }

  /** @return current arm angle in radians (sourced from IO inputs) */
  public double getAngleRadians() {
    return m_inputs.angleRadians;
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // 1. Refresh sensor snapshot from hardware / sim
    m_armIO.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);

    // 2. Calculate control output
    double feedbackOutputVolts = m_feedbackController.calculate(m_inputs.angleRadians);
    double feedforwardOutputVolts =
        m_feedforward.calculate(
            m_feedbackController.getSetpoint().position,
            m_feedbackController.getSetpoint().velocity);
    double totalAppliedVolts = feedbackOutputVolts + feedforwardOutputVolts;

    // 3. Send voltage command to hardware / sim
    m_armIO.setVoltage(totalAppliedVolts);

    // 4. Telemetry
    Logger.recordOutput("Arm/AngleDegrees", Math.toDegrees(m_inputs.angleRadians));
    Logger.recordOutput(
        "Arm/VelocityDegreesPerSec", Math.toDegrees(m_inputs.velocityRadiansPerSecond));
    Logger.recordOutput("Arm/GoalDegrees", Math.toDegrees(m_goalAngleRadians));
    Logger.recordOutput("Arm/AppliedOutputVolts", totalAppliedVolts);
    Logger.recordOutput("Arm/CurrentAmps", m_inputs.currentAmps);
    Logger.recordOutput("Arm/AtGoal", atGoal());
    Logger.recordOutput("Arm/ForwardLimitSwitch", m_inputs.forwardLimitSwitchTripped);
    Logger.recordOutput("Arm/ReverseLimitSwitch", m_inputs.reverseLimitSwitchTripped);

    SmartDashboard.putNumber("Arm/Angle (deg)", Math.toDegrees(m_inputs.angleRadians));
    SmartDashboard.putNumber(
        "Arm/Velocity (deg per sec)", Math.toDegrees(m_inputs.velocityRadiansPerSecond));
    SmartDashboard.putNumber("Arm/Goal (deg)", Math.toDegrees(m_goalAngleRadians));
    SmartDashboard.putNumber("Arm/Applied Output (V)", totalAppliedVolts);
    SmartDashboard.putNumber("Arm/Current (A)", m_inputs.currentAmps);
    SmartDashboard.putBoolean("Arm/At Goal", atGoal());
    SmartDashboard.putBoolean("Arm/Forward Limit Switch", m_inputs.forwardLimitSwitchTripped);
    SmartDashboard.putBoolean("Arm/Reverse Limit Switch", m_inputs.reverseLimitSwitchTripped);
  }
}
