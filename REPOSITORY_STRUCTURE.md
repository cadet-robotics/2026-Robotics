## Repository structure: 2026-Robotics

This document describes the purpose of the top-level files, folders, and the main modules in this repository so you can quickly find where features, configuration, and build outputs live.

### Quick summary

- Language / framework: Java (WPILib robotics project). Gradle is used for build automation.
- Source code: `src/main/java` (robot code lives in `frc.robot`).
- Deployment/runtime assets and configuration: `src/main/deploy` (PathPlanner and swerve JSONs).

---

## Top-level files

- `build.gradle` — Gradle build script for the project. Contains dependency and task configuration.
- `settings.gradle` — Gradle settings for the multi-module or project name configuration.
- `gradlew`, `gradlew.bat` — Gradle wrapper scripts (Unix/Windows). Use these to run Gradle without a local installation.
- `networktables.json`, `simgui*.json` — Simulation and network configuration files used by the sim tooling.
- `can_bus.md`, `limelightpipelines.md`, `WPILib-License.md` — Documentation files: CAN bus layout notes, Limelight pipeline notes, and WPILib license.
- `todo.md` — Project TODOs and notes.

## Important top-level directories

- `src/` — The canonical source tree. See detailed breakdown below.
- `vendordeps/` — JSON manifests describing vendor / library dependency bundles (e.g., Photon, REV, PathPlanner). Useful for dependency management and reference.
- `build/` — Gradle build outputs: compiled classes, generated sources, jars, test outputs, and other build artifacts. Do not edit manually.
- `bin/` — Prebuilt or example binaries/deploy artifacts for the robot and simulator.
- `generated/` — Generated sources and headers created by annotation processors or build steps.
- `jni/` — Native library artifacts (if present) for the simulator or robot native code.
- `libs/` — External jars or libraries referenced by the project (local copies).
- `reports/` — Test and static analysis reports (e.g., Problems report HTML).
- `tmp/` — Temporary files created during compilation.

---

## `src/` layout (main area of interest)

Path: `src/main/`

- `deploy/` — contains runtime configuration files used when deploying or simulating the robot. Notable subfolders:
  - `pathplanner/` — PathPlanner settings, paths, and autos definitions.
  - `swerve/` — Swerve drive configuration files (controller properties and per-module JSONs).
  - `example.txt` — example deploy content or notes.

- `java/frc/robot/` — primary Java source package. Key files and directories:

  - `Autos.java` — Autonomous routine definitions and sequences used by the robot during autonomous periods.

  - `Configs.java` — Centralized configuration classes and helper data structures used to initialize subsystems (examples in build outputs show inner classes like `Configs$DriveSubsystem`).

  - `Constants.java` — Project constants (numbers, ports, tuning parameters) used across subsystems and commands.

  - `Main.java` — Program entrypoint that bootstraps the robot application (WPILib main).

  - `Robot.java` — Main robot lifecycle class (init, periodic, disabled, teleop, autonomous hooks) managed by WPILib.

  - `RobotContainer.java` — Responsible for wiring subsystems, commands, button bindings, and autonomous chooser. Typical single place to see the robot's control configuration.

  - `Libs/` — Reusable library helpers for this project. Files include:
    - `CCommand.java` — Likely a convenience wrapper or helper for WPILib commands (project-specific command utilities).
    - `CSubsystem.java` — Base or helper class for subsystems (common functionality across subsystems).
    - `LimelightHelpers.java` — Utility functions to interact with the Limelight vision camera.

  - `Subsystems/` — Implementation of robot systems. Files include:
    - `Drive.java` — Swerve or drivetrain subsystem implementation (kinematics, module initialization, drive control).
    - `Indexer.java` — Ball indexing subsystem (moves game pieces inside robot to/from shooter/intake).
    - `Intake.java` — Intake subsystem for acquiring game pieces from the field.
    - `Shooter.java` — Shooter subsystem for launching game pieces.
    - `Vision/` — Vision-related classes:
      - `Vision.java` — Common vision interface or abstract helper used by both simulation and real hardware.
      - `RealVision.java` — Real-hardware vision implementation that talks to Limelight / network tables.
      - `SimVision.java` — Simulation-specific vision implementation used when running in sim environment.

---

## `vendordeps/`

This folder holds JSON manifests produced or consumed by the vendor dependency manager used by WPILib / vendordeps tooling. Files such as `photonlib.json`, `REVLib.json`, `PathplannerLib-2026.1.2.json` list versions and metadata for third-party libraries used on robot and simulator.

## `build/` (details)

- `build/classes/java/main/frc/robot/...` — compiled `.class` files corresponding to the Java sources. The build tree shows compiled classes for `Configs`, `Drive`, `RealVision`, `SimVision`, etc.
- `build/libs/` — produced JARs for the project if packaging is performed.
- `build/reports/` — check for static analysis and test reports.

---

## Notes about code organization and development touchpoints

- If you want to add or modify robot behavior, start in `src/main/java/frc/robot/Subsystems` and `RobotContainer.java` (bindings and command chooser).
- To change swerve configuration, adjust the JSONs under `src/main/deploy/swerve/modules` and `src/main/deploy/swerve/*.json` and update any parsing logic in `Drive.java` or `Configs.java`.
- Use `RealVision.java` for Limelight integrations on real hardware and `SimVision.java` for simulation-specific logic; both implement or use `Vision.java` as the common contract.
- Check `CCommand.java` and `CSubsystem.java` to learn patterns the project uses for commands and subsystems; reuse those helpers for new features for consistency.

## How to build (local)

From the repository root on a machine with Java and the Gradle wrapper available, run the wrapper to build (PowerShell example):

```powershell
./gradlew build
```

Or on Windows explicitly:

```powershell
.\gradlew.bat build
```

That will populate the `build/` directory with compiled classes and artifacts.

## Where to look next (quick pointers)

- To inspect autonomous paths: `src/main/deploy/pathplanner/`.
- To find the code that reads swerve module JSON: search for references to `swerve`/`modules` in `Configs.java` or `Drive.java`.
- For simulation configuration: check `simgui-*.json` files at repo root.

---

If you'd like, I can also:
- generate a small README-style index with links to the most important classes,
- or open `RobotContainer.java` and annotate where commands and subsystems are bound.

End of document.
