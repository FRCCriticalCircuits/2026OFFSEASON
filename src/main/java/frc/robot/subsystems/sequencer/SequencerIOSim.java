// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sequencer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.SequencerConstants;

/**
 * WPILib simulation implementation of {@link SequencerIO}.
 *
 * <p>Uses {@link ElevatorSim} to model the sequencer dynamics in simulation.
 */
public class SequencerIOSim implements SequencerIO {

  // ─── Simulation plant ──────────────────────────────────────────────────────

  private final ElevatorSim m_sequencerSimulation =
      new ElevatorSim(
          DCMotor.getKrakenX60(2), // two Kraken X60 motors driving the sequencer
          SequencerConstants.kGearRatio,
          SequencerConstants.kCarriageMassKilograms,
          SequencerConstants.kDrumRadiusMeters,
          SequencerConstants.kMinHeightMeters,
          SequencerConstants.kMaxHeightMeters,
          true, // simulate gravity
          0.0 // starting height (meters)
          );

  // ─── Applied output tracking ───────────────────────────────────────────────

  private double m_appliedVolts = 0.0;

  // ─── SequencerIO ───────────────────────────────────────────────────────────

  @Override
  public void updateInputs(SequencerIOInputs inputs) {
    // Advance the simulation by one 20 ms robot loop
    m_sequencerSimulation.setInput(m_appliedVolts);
    m_sequencerSimulation.update(0.020);

    inputs.heightMeters            = m_sequencerSimulation.getPositionMeters();
    inputs.velocityMetersPerSecond = m_sequencerSimulation.getVelocityMetersPerSecond();
    inputs.appliedVolts            = m_appliedVolts;
    inputs.currentAmps             = m_sequencerSimulation.getCurrentDrawAmps();
    inputs.lowerLimitSwitchTripped = m_sequencerSimulation.hasHitLowerLimit();
    inputs.upperLimitSwitchTripped = m_sequencerSimulation.hasHitUpperLimit();
  }

  @Override
  public void setVoltage(double appliedVolts) {
    // Clamp to typical 12-V battery
    m_appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
  }

  @Override
  public void resetEncoder() {
    // Limit-switch behavior handles auto-zeroing in this sim implementation.
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    // Hardware-only concept
  }
}
