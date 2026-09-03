// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;

/**
 * WPILib simulation implementation of {@link ArmIO}.
 *
 * <p>Uses {@link SingleJointedArmSim} to model the arm dynamics in simulation.
 */
public class ArmIOSim implements ArmIO {

  // ─── Simulation plant ──────────────────────────────────────────────────────

  /** Moment of inertia computed from mass and length: I = (1/3) * m * L^2. */
  private static final double kMomentOfInertiaKgMetersSquared =
      SingleJointedArmSim.estimateMOI(
          ArmConstants.kArmLengthMeters,
          ArmConstants.kArmMassKilograms);

  private final SingleJointedArmSim m_armSimulation =
      new SingleJointedArmSim(
          DCMotor.getNeoVortex(1),
          ArmConstants.kGearRatio,
          kMomentOfInertiaKgMetersSquared,
          ArmConstants.kArmLengthMeters,
          ArmConstants.kMinAngleRadians,
          ArmConstants.kMaxAngleRadians,
          true, // simulate gravity
          0.0   // starting angle (radians)
      );

  // ─── Applied output tracking ───────────────────────────────────────────────

  private double m_appliedVolts = 0.0;

  // ─── ArmIO ────────────────────────────────────────────────────────────────

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Advance the simulation by one 20 ms robot loop
    m_armSimulation.setInput(m_appliedVolts);
    m_armSimulation.update(0.020);

    inputs.angleRadians              = m_armSimulation.getAngleRads();
    inputs.velocityRadiansPerSecond  = m_armSimulation.getVelocityRadPerSec();
    inputs.appliedVolts              = m_appliedVolts;
    inputs.currentAmps               = m_armSimulation.getCurrentDrawAmps();
    inputs.forwardLimitSwitchTripped = m_armSimulation.hasHitUpperLimit();
    inputs.reverseLimitSwitchTripped = m_armSimulation.hasHitLowerLimit();
  }

  @Override
  public void setVoltage(double appliedVolts) {
    // Clamp to typical 12-V battery
    m_appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
  }

  @Override
  public void resetEncoder() {
    // No-op for sim purposes
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    // Hardware-only concept
  }
}
