# Dual Kraken X60 Intake Roller Implementation Handoff

## Summary of Changes
1. **Constants Configuration (`src/main/java/frc/robot/Constants.java`)**:
   - Added `kLeaderMotorId = 31` and `kFollowerMotorId = 32` under `RollerConstants`.
   - Maintained `kMotorId = kLeaderMotorId` alias for backward compatibility.
   - Retained stator (60A) and supply (60A) current limit configurations.

2. **Hardware IO Abstraction (`src/main/java/frc/robot/subsystems/roller/RollerIO.java`)**:
   - Updated `RollerIOInputs` with `leaderCurrentAmps` and `followerCurrentAmps` alongside `velocityRotationsPerSecond`, `appliedVolts`, `currentAmps`, and `gamePieceDetected`.

3. **Kraken IO Implementation (`src/main/java/frc/robot/subsystems/roller/RollerIOKraken.java`)**:
   - Added constructor `RollerIOKraken(int leaderCanId, int followerCanId)` and 1-arg overload `RollerIOKraken(int leaderCanId)`.
   - Configured both TalonFX motors on the CANivore bus with stator and supply current limits.
   - Set follower motor to follow leader with `Follower(leaderCanId, MotorAlignmentValue.Aligned)`.
   - Synchronized brake/coast mode toggles across both leader and follower motors.
   - Updated `updateInputs()` to read both leader and follower stator currents and applied voltage.

4. **Simulation Physics Model (`src/main/java/frc/robot/subsystems/roller/RollerIOSim.java`)**:
   - Updated `FlywheelSim` plant model to model 2 Kraken X60 motors via `DCMotor.getKrakenX60(2)`.
   - Distributed total current draw equally between `leaderCurrentAmps` and `followerCurrentAmps`.

5. **Subsystem Telemetry & Getters (`src/main/java/frc/robot/subsystems/roller/Roller.java`)**:
   - Added getters `getLeaderCurrentAmps()`, `getFollowerCurrentAmps()`, and `getAppliedVolts()`.
   - Published SmartDashboard telemetry keys: `Roller/Velocity (RPS)`, `Roller/Applied Output (V)`, `Roller/Leader Current (A)`, `Roller/Follower Current (A)`, `Roller/Current (A)`, `Roller/Game Piece Detected`.

6. **Robot Container Wiring (`src/main/java/frc/robot/RobotContainer.java`)**:
   - Updated `RollerIOKraken` initialization to pass `RollerConstants.kLeaderMotorId` and `RollerConstants.kFollowerMotorId`.

7. **Documentation & Tests (`TODO.md`, `SubsystemsTest.java`)**:
   - Updated CAN ID mappings and tuning table in `TODO.md`.
   - Enhanced `SubsystemsTest.java` with comprehensive assertions on intake, outtake, hold, stop, voltage, and dual current telemetry, plus command factories.

## Verification
- `./gradlew test` ran 26 unit tests across all test suites, passing with 0 failures.
- `./gradlew build` completed successfully.
