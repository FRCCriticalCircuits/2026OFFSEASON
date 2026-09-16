// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.sequencer.Sequencer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.superstructure.SuperstructureState.RollerAction;
import frc.robot.superstructure.SuperstructureState.SequencerAction;
import frc.robot.superstructure.SuperstructureState.ShooterAction;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoAim.AutoAimResult;
import java.util.function.DoubleSupplier;

/**
 * The Superstructure coordinates the Sequencer feeder, Arm, Roller, and combined Shooter (Flywheel + Hood)
 * subsystems using an enum-based state machine and dynamic Auto-Aim automation.
 */
public class Superstructure extends SubsystemBase {

  // ─── Subsystems ────────────────────────────────────────────────────────────

  private final Sequencer m_sequencer;
  private final Arm       m_arm;
  private final Roller    m_roller;
  private final Shooter   m_shooter;

  // ─── State machine ─────────────────────────────────────────────────────────

  private SuperstructureState m_currentState = SuperstructureState.STOW;
  private SuperstructureState m_desiredState  = SuperstructureState.STOW;

  // ─── Constructor ───────────────────────────────────────────────────────────

  public Superstructure(Sequencer sequencer, Arm arm, Roller roller, Shooter shooter) {
    m_sequencer = sequencer;
    m_arm       = arm;
    m_roller    = roller;
    m_shooter   = shooter;
  }

  // ─── Public Commands ───────────────────────────────────────────────────────

  /**
   * Returns a {@link Command} that transitions the superstructure to {@code targetState}
   * and finishes once mechanisms reach their goals.
   *
   * @param targetState the desired {@link SuperstructureState}
   */
  public Command setStateCommand(SuperstructureState targetState) {
    return Commands.runOnce(() -> {
          m_desiredState = targetState;
          applyState(targetState);
        }, this)
        .andThen(Commands.waitUntil(this::atDesiredState))
        .withName("Superstructure → " + targetState.name());
  }

  /**
   * Returns a {@link Command} that holds the superstructure at {@code targetState}
   * indefinitely (until cancelled). Useful for teleop triggers.
   *
   * @param targetState the desired {@link SuperstructureState}
   */
  public Command holdStateCommand(SuperstructureState targetState) {
    return Commands.run(() -> {
          m_desiredState = targetState;
          applyState(targetState);
        }, this)
        .withName("Hold → " + targetState.name());
  }

  /**
   * Sequential intake command:
   * 1. Deploys arm to ground intake position with roller and sequencer stopped.
   * 2. Waits until arm reaches the target angle.
   * 3. Runs the roller to intake balls while held.
   * 4. Automatically returns arm to STOW and stops the roller on release.
   */

  public Command manual_intake()
  {
    return Commands.sequence(
      Commands.runOnce(() -> {
        m_arm.setGoal(Math.toRadians(15));
        m_roller.runIntake();
      }, this)
    ).finallyDo(
        () -> {
        m_arm.setGoal(Math.toRadians(0));
        m_roller.stop(); 
        }
      ).withName("Manual Intake");
  }
  public Command intakeSequenceCommand() {
    return Commands.sequence(
        // Step 1: Move arm to ground target angle while roller and sequencer are stopped
        Commands.runOnce(() -> {
          m_desiredState = SuperstructureState.INTAKE_GROUND;
          m_arm.setGoal(SuperstructureState.INTAKE_GROUND.armAngleRadians);
          m_roller.stop();
          m_sequencer.stop();
        }, this),

        // Step 2: Wait until arm reaches target angle
        Commands.waitUntil(m_arm::atGoal),

        // Step 3: Run roller to intake balls
        Commands.run(() -> m_roller.runIntake(), this)
    ).finallyDo(interrupted -> {
        // Step 4: Retract arm to STOW and stop roller & sequencer
        applyState(SuperstructureState.STOW);
        m_desiredState = SuperstructureState.STOW;
    }).withName("Superstructure.intakeSequence");
  }

  /**
   * Dynamic Auto-Aim & Shoot Command:
   * 1. Continuously tracks distance to goal and aligns swerve heading while driver translates.
   * 2. Dynamically calculates and applies flywheel speed, hood angle, and supporting shooter from distance.
   * 3. Once heading and shooter are locked (arm pivot motor is NOT used), spins sequencer to feed balls.
   * 4. Automatically stops shooter, sequencer, and roller when released without altering arm position.
   *
   * @param swerve Swerve drivetrain subsystem.
   * @param xSpeedSupplier Driver X translation supplier.
   * @param ySpeedSupplier Driver Y translation supplier.
   */
  public Command autoAimAndShootCommand(
      SwerveDrive swerve,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier) {
    return Commands.run(
        () -> {
          AutoAimResult aimResult = AutoAim.calculate(swerve.getPose());

          // 1. Swerve heading alignment with manual driver translation (reduced by 30% for safety)
          double autoAimMaxSpeedMetersPerSecond =
              SwerveConstants.kMaxSpeedMetersPerSecond * AutoAimConstants.kAutoAimMaxSpeedMultiplier;

          swerve.driveWithHeadingLock(
              xSpeedSupplier.getAsDouble() * autoAimMaxSpeedMetersPerSecond,
              ySpeedSupplier.getAsDouble() * autoAimMaxSpeedMetersPerSecond,
              aimResult.targetHeading,
              true);

          // 2. Set dynamic shooter parameters from distance
          m_shooter.prepareShot(
              aimResult.flywheelVelocityRotationsPerSecond,
              aimResult.hoodAngleRadians);

          // 3. Feed balls when on target (arm pivot motor is NOT used for auto-aim)
          boolean fullyReady =
              aimResult.headingAligned
                  && m_shooter.isReadyToShoot();

          if (fullyReady) {
            m_sequencer.feed();   // Spin sequencer to feed all balls into shooter
            m_roller.runIntake(); // Assist ball feed
            m_desiredState = SuperstructureState.SHOOT;
          } else {
            m_sequencer.stop();   // Hold balls in sequencer until ready
            m_roller.runHold();
            m_desiredState = SuperstructureState.SPIN_UP_SHOOT;
          }

          SmartDashboard.putNumber("AutoAim/Target Distance (m)", aimResult.distanceMeters);
          SmartDashboard.putNumber("AutoAim/Target Heading (deg)", aimResult.targetHeading.getDegrees());
          SmartDashboard.putBoolean("AutoAim/Heading Aligned", aimResult.headingAligned);
          SmartDashboard.putBoolean("AutoAim/Ready To Fire", fullyReady);
        },
        this,
        swerve).finallyDo(interrupted -> {
          // Stop shooter, sequencer, and roller upon trigger release (leave arm untouched)
          m_shooter.stop();
          m_sequencer.stop();
          m_roller.stop();
          m_desiredState = SuperstructureState.STOW;
        }).withName("Superstructure.autoAimAndShoot");
  }

  public Command manual_shoot() {
    return Commands.run(
        () -> {
          m_shooter.prepareShot(20, Math.toRadians(30));
          m_sequencer.setVelocity(10);
          m_desiredState = SuperstructureState.SPIN_UP_SHOOT;
        },
        this)
        .finallyDo(interrupted -> {
          m_shooter.stop();
          m_sequencer.stop();
          m_desiredState = SuperstructureState.STOW;
        })
        .withName("Superstructure.manualShoot");
  }

  /** @return the state the s
   * uperstructure is currently transitioning toward */
  public SuperstructureState getDesiredState() {
    return m_desiredState;
  }

  /** @return the last fully-reached state */
  public SuperstructureState getCurrentState() {
    return m_currentState;
  }

  /** @return true when arm has reached the desired state goal */
  public boolean atDesiredState() {
    boolean armAtGoal = m_arm.atGoal();
    if (m_desiredState == SuperstructureState.SHOOT || m_desiredState == SuperstructureState.SPIN_UP_SHOOT) {
      return armAtGoal && m_shooter.isReadyToShoot();
    }
    return armAtGoal;
  }

  // ─── State application ─────────────────────────────────────────────────────

  /**
   * Applies the setpoints for a given state to all subsystems.
   */
  private void applyState(SuperstructureState targetState) {
    m_arm.setGoal(targetState.armAngleRadians);
    applyRollerAction(targetState.rollerAction);
    applyShooterAction(targetState.shooterAction);
    applySequencerAction(targetState.sequencerAction);
  }

  private void applyRollerAction(RollerAction action) {
    switch (action) {
      case INTAKE -> m_roller.runIntake();
      case HOLD   -> m_roller.runHold();
      case STOP   -> m_roller.stop();
    }
  }

  private void applyShooterAction(ShooterAction action) {
    switch (action) {
      case SHOOT, SPIN_UP -> {
        m_shooter.runFlywheel();
        m_shooter.runSupportingShooter();
      }
      case IDLE           -> {
        m_shooter.runIdleFlywheel();
        m_shooter.stowHood();
      }
      case STOP           -> m_shooter.stop();
    }
  }

  private void applySequencerAction(SequencerAction action) {
    switch (action) {
      case FEED -> m_sequencer.feed();
      case STOP -> m_sequencer.stop();
    }
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // Promote desired → current when we arrive
    if (atDesiredState()) {
      m_currentState = m_desiredState;
    }

    Logger.recordOutput("Superstructure/CurrentState", m_currentState.name());
    Logger.recordOutput("Superstructure/DesiredState", m_desiredState.name());
    Logger.recordOutput("Superstructure/AtGoal", atDesiredState());

    SmartDashboard.putString("Superstructure/Current State", m_currentState.name());
    SmartDashboard.putString("Superstructure/Desired State", m_desiredState.name());
    SmartDashboard.putBoolean("Superstructure/At Goal",      atDesiredState());
  }
}
