# Dispatch Log

## 2026-08-31T23:23:37Z

Task:
Add a second Kraken X60 motor (Intake Roller Motor 2) to the intake roller subsystem, configuring Phoenix 6 hardware IO with aligned follower mode on CAN ID 32, simulation physics, telemetry, and subsystem interfaces.

Requirements:
### R1. Hardware Constants Configuration
- In `src/main/java/frc/robot/Constants.java` under `RollerConstants`, define constants for both intake roller motors:
  - Leader Motor CAN ID: `kLeaderMotorId = 31` (or `kMotorId = 31`)
  - Follower Motor CAN ID: `kFollowerMotorId = 32` (or `kMotor2Id = 32`)
  - Maintain stator and supply current limit configurations for both Kraken X60 motors.

### R2. Hardware IO Abstraction & Kraken Implementation
- In `src/main/java/frc/robot/subsystems/roller/RollerIO.java`:
  - Update `RollerIOInputs` to report telemetry for both leader and follower motors (velocity, applied volts, leader current in Amps, follower current in Amps, and game piece detected status).
- In `src/main/java/frc/robot/subsystems/roller/RollerIOKraken.java`:
  - Accept both `leaderCanId` and `followerCanId` in the constructor.
  - Instantiate and configure both TalonFX motors on the CANivore bus (`Constants.kCANBusName`).
  - Configure the follower motor with identical current limits, coast neutral mode, and configure follower control using `Follower(leaderCanId, MotorAlignmentValue.Aligned)` (or matching control mode).
  - Synchronize brake/coast mode toggles across both motors.
  - Populate both leader and follower currents/voltages in `updateInputs()`.

### R3. Physics Simulation, Subsystem Telemetry & Wiring
- In `src/main/java/frc/robot/subsystems/roller/RollerIOSim.java`:
  - Update the `FlywheelSim` plant model to reflect 2 Kraken X60 motors (`DCMotor.getKrakenX60(2)`).
- In `src/main/java/frc/robot/subsystems/roller/Roller.java`:
  - Publish telemetry for both leader and follower motors to SmartDashboard (e.g. `Roller/Leader Current (A)`, `Roller/Follower Current (A)`).
- In `src/main/java/frc/robot/RobotContainer.java`:
  - Update the `RollerIOKraken` constructor call to pass both `RollerConstants.kLeaderMotorId` and `RollerConstants.kFollowerMotorId`.
- In `src/test/java/frc/robot/SubsystemsTest.java`:
  - Verify that roller subsystem tests pass and reflect the updated subsystem setup.

Execute the SWE Light workflow with implementer and reviewer(s), verify with test suite (`./gradlew test` / `./gradlew build`), and report completion back to the Sentinel.
