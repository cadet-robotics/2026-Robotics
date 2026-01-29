# 2026-Robotics — Quick index

Short guide to the repository and where to find the main robot code.

## Quick facts

- Language: Java (WPILib robot project)
- Build: Gradle (use the included Gradle wrapper)
- Source: `src/main/java/frc/robot`

## Most important files

- `src/main/java/frc/robot/Robot.java` — robot lifecycle (init/periodic hooks).
- `src/main/java/frc/robot/RobotContainer.java` — wiring of subsystems, commands, and controller bindings (central place to add button mappings).
- `src/main/java/frc/robot/Subsystems/Drive.java` — drivetrain/swerve subsystem.
- `src/main/java/frc/robot/Subsystems/Intake.java` — intake subsystem.
- `src/main/java/frc/robot/Subsystems/Shooter.java` — shooter subsystem.
- `src/main/java/frc/robot/Subsystems/Indexer.java` — indexer that coordinates balls between subsystems.
- `src/main/java/frc/robot/Autos.java` — autonomous sequence definitions.
- `src/main/java/frc/robot/Configs.java` and `Constants.java` — configuration, ports and tuning constants.
- `src/main/deploy/` — JSON assets for PathPlanner and swerve modules used at deploy/sim time.

## Build & deploy (local)

From repository root (Windows PowerShell):

```powershell
.\gradlew.bat build
.\gradlew.bat deploy
```

Replace `deploy` with the specific Gradle target you need (e.g., `deploy -Proborio=10.68.68.2` depending on your setup).

## Where to start making changes

- To add new command bindings: edit `RobotContainer.java`.
- To change swerve module tuning: edit files in `src/main/deploy/swerve/` and the parsing logic in `Configs.java` / `Drive.java`.
- To add autonomous paths: add PathPlanner files in `src/main/deploy/pathplanner/` and register them in `Autos.java`.

## Notes

- A companion file `ANNOTATED_ROBOTCONTAINER.md` is included with a line-by-line walkthrough of `RobotContainer.java` and guidance for wiring subsystems and commands.
