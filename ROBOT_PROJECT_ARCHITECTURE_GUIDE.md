# FRC 2026 Robot Project - Complete Architecture Guide

## Table of Contents
1. [Project Overview](#project-overview)
2. [Architecture Pattern](#architecture-pattern)
3. [Core Files](#core-files)
4. [Subsystem Deep Dive](#subsystem-deep-dive)
5. [Custom Framework](#custom-framework)
6. [Configuration System](#configuration-system)
7. [Code Flow Examples](#code-flow-examples)
8. [Best Practices](#best-practices)

---

## Project Overview

This is a **2026 FRC robot project** using the **Command-Based Programming** paradigm with custom enhancements. The robot features:

- **Swerve Drive** - Holonomic drive system using YAGSL (Yet Another Swerve Library)
- **Shooter** - Dual flywheel mechanism for launching game pieces
- **Intake** - Collection mechanism for acquiring game pieces
- **Indexer** - Transfer mechanism between intake and shooter
- **Vision** - AprilTag detection and pose estimation
- **Autonomous** - PathPlanner integration for complex path following

**Key Libraries:**
- WPILib 2026 - Official FRC framework
- YAGSL - Swerve drive management
- YAMS - Motor controller abstraction
- PathPlanner - Autonomous path planning
- Maple Sim - Physics simulation

---

## Architecture Pattern

### The Command-Based Architecture

```
┌─────────────────────────────────────────────────────────┐
│                      Robot.java                         │
│  (Entry point - manages mode lifecycle)                 │
└────────────────────┬────────────────────────────────────┘
                     │ Creates
                     ▼
┌─────────────────────────────────────────────────────────┐
│                  RobotContainer.java                    │
│  (Central hub - creates all subsystems & bindings)      │
└─────┬──────────┬──────────┬──────────┬─────────────────┘
      │          │          │          │
      ▼          ▼          ▼          ▼
┌──────────┐ ┌─────────┐ ┌─────────┐ ┌──────────┐
│  Drive   │ │ Shooter │ │ Intake  │ │ Indexer  │
│Subsystem │ │Subsystem│ │Subsystem│ │Subsystem │
└──────────┘ └─────────┘ └─────────┘ └──────────┘
      │          │          │          │
      └──────────┴──────────┴──────────┘
                     │
              Returns Commands
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              CommandScheduler                           │
│  (Runs in robotPeriodic() - executes all commands)     │
└─────────────────────────────────────────────────────────┘
```

### Class Hierarchy

```
SubsystemBase (WPILib)
    ↓
CSubsystem (Custom base class)
    ↓
Drive, Shooter, Intake, Indexer (Concrete subsystems)

Command (WPILib)
    ↓
CCommand (Custom command builder)
    ↓
All robot commands
```

---

## Core Files

### 1. Main.java
**Purpose:** Application entry point
**Location:** `src/main/java/frc/robot/Main.java`

```java
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot application entry point.
 * DO NOT MODIFY unless you know what you're doing.
 */
public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
```

**Flow:**
1. JVM starts `main()` method
2. Creates `Robot` instance
3. Enters WPILib framework control loop

---

### 2. Robot.java
**Purpose:** Lifecycle manager for the robot
**Location:** `src/main/java/frc/robot/Robot.java`

**Key Concepts:**

```java
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    
    public Robot() {
        // Initialize simulation arena BEFORE subsystems
        if (isSimulation()) {
            SimulatedArena.getInstance();
        }
        // Create all subsystems and bindings
        this.robotContainer = new RobotContainer();
    }
    
    // ═══════════════════════════════════════════════════
    // RUNS EVERY 20ms REGARDLESS OF MODE
    // ═══════════════════════════════════════════════════
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();  // THE HEART OF THE ROBOT
    }
    
    // ═══════════════════════════════════════════════════
    // DISABLED MODE (Robot powered but not enabled)
    // ═══════════════════════════════════════════════════
    @Override
    public void disabledInit() { /* Called once when disabled */ }
    
    @Override
    public void disabledPeriodic() { /* Called every 20ms while disabled */ }
    
    // ═══════════════════════════════════════════════════
    // AUTONOMOUS MODE (15 seconds, no driver control)
    // ═══════════════════════════════════════════════════
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        // Auto command gets scheduled
    }
    
    @Override
    public void autonomousPeriodic() { /* Usually empty */ }
    
    // ═══════════════════════════════════════════════════
    // TELEOP MODE (2min 15sec, driver control)
    // ═══════════════════════════════════════════════════
    @Override
    public void teleopInit() {
        // Cancel auto if still running
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
    
    @Override
    public void teleopPeriodic() { /* Usually empty */ }
}
```

**Critical:** `robotPeriodic()` runs the `CommandScheduler`, which executes all active commands every 20ms.

---

### 3. RobotContainer.java
**Purpose:** The "hub" - creates subsystems, controllers, and bindings
**Location:** `src/main/java/frc/robot/RobotContainer.java`

```java
public class RobotContainer {
    // ═══════════════════════════════════════════════════
    // SUBSYSTEM DECLARATIONS
    // ═══════════════════════════════════════════════════
    private final Drive drive_subsystem;
    private final Vision vision_subsystem;
    // private final Shooter shooter_subsystem;  // Currently commented out
    // private final Intake intake_subsystem;
    // private final Indexer indexer_subsystem;
    
    // ═══════════════════════════════════════════════════
    // CONTROLLER DECLARATIONS
    // ═══════════════════════════════════════════════════
    private final CommandXboxController driverController;
    // private final CommandXboxController codriverController;
    
    private final Autos autos;
    
    public RobotContainer() {
        // 1. CREATE SUBSYSTEMS (order matters for dependencies)
        drive_subsystem = new Drive();
        vision_subsystem = drive_subsystem.getVision();
        // shooter_subsystem = new Shooter();
        // intake_subsystem = new Intake();
        // indexer_subsystem = new Indexer(shooter_subsystem, intake_subsystem);
        
        // 2. CREATE AUTONOMOUS HANDLER
        autos = new Autos(this, drive_subsystem);
        
        // 3. CREATE CONTROLLERS
        driverController = new CommandXboxController(0);
        
        // 4. CONFIGURE BUTTON BINDINGS
        configureBindings();
        
        // 5. SET DEFAULT COMMANDS
        configureDriving();
    }
    
    // ═══════════════════════════════════════════════════
    // BUTTON BINDINGS (Map buttons to commands)
    // ═══════════════════════════════════════════════════
    private void configureBindings() {
        // Example: A button drives to a specific pose
        // driverController.a().whileTrue(
        //     drive_subsystem.dummyDrivePose()
        // );
        
        // Example: Shooter controls
        // codriverController.leftTrigger()
        //     .whileTrue(shooter_subsystem.Shoot())
        //     .onFalse(shooter_subsystem.StopShooting());
    }
    
    // ═══════════════════════════════════════════════════
    // DEFAULT DRIVE COMMAND
    // ═══════════════════════════════════════════════════
    public void configureDriving() {
        // Create suppliers that read controller input
        DoubleSupplier getTranslationX = () -> 
            MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
        DoubleSupplier getTranslationY = () -> 
            MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
        DoubleSupplier getHeadingX = () -> -driverController.getRightX();
        DoubleSupplier getHeadingY = () -> -driverController.getRightY();
        
        // Create command that reads these suppliers every cycle
        Command defaultDrive = drive_subsystem.driveCommand(
            getTranslationX, getTranslationY, getHeadingX, getHeadingY
        );
        
        // Set as default (runs whenever no other command uses drive)
        drive_subsystem.setDefaultCommand(defaultDrive);
    }
    
    public Command getAutonomousCommand() {
        return null;  // Currently disabled
    }
}
```

**Key Points:**
- Creates subsystems in **constructor order** (dependencies first)
- Maps Xbox controller buttons to commands
- Sets **default commands** that run continuously
- Uses `DoubleSupplier` for real-time joystick values

---

## Subsystem Deep Dive

### Drive Subsystem
**Location:** `src/main/java/frc/robot/Subsystems/Drive.java`

**Purpose:** Controls swerve drive system with vision integration

```java
public class Drive extends CSubsystem {
    private static final double maxSpeed = Units.feetToMeters(6);
    private SwerveDrive swerveDrive;
    private Vision vision;
    
    public Drive() {
        // 1. Initialize vision (real or simulated)
        if (Robot.isReal()) {
            vision = new RealVision(this);
        } else {
            // Simulation vision would go here
        }
        
        // 2. Set starting pose
        Pose2d startingPose = new Pose2d(
            new Translation2d(Meter.of(1), Meter.of(4)),
            Rotation2d.fromDegrees(180)
        );
        
        // 3. Load swerve configuration from JSON files
        configureSwerveObjects(startingPose);
        
        // 4. Configure PathPlanner autonomous
        DriveSubsystemConfiguration.configurePathPlanner(this, swerveDrive);
    }
    
    /**
     * Loads swerve drive configuration from deploy/swerve/*.json
     */
    public void configureSwerveObjects(Pose2d startingPose) {
        File swerveJsonDirectory = new File(
            Filesystem.getDeployDirectory(), 
            "swerve"
        );
        
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory)
                .createSwerveDrive(Drive.maxSpeed, startingPose);
        } catch (IOException e) {
            throw new RuntimeException(
                "Failed to load swerve configuration: " + e
            );
        }
        
        swerveDrive.resetOdometry(startingPose);
        SwerveDriveTelemetry.verbosity = 
            SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }
}
```

**Key Methods:**

#### driveCommand() - Default Driving
```java
public Command driveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier headingX,
    DoubleSupplier headingY
) {
    return cCommand("Drive")
        .onExecute(() -> {
            // Read joystick values every 20ms
            double xVelocity = translationX.getAsDouble() * maxSpeed;
            double yVelocity = translationY.getAsDouble() * maxSpeed;
            double angularVelocity = calculateAngularVelocity(
                headingX.getAsDouble(), 
                headingY.getAsDouble()
            );
            
            // Drive the robot
            swerveDrive.drive(
                new Translation2d(xVelocity, yVelocity),
                angularVelocity,
                true,  // Field-relative
                false  // Not open loop
            );
        });
}
```

#### updatePose() - Vision Integration
```java
public void updatePose(Pose2d visionPose) {
    if (visionPose != null) {
        swerveDrive.addVisionMeasurement(
            visionPose,
            Timer.getFPGATimestamp()
        );
    }
}
```

**Vision updates the robot's estimated position using AprilTags.**

#### driveToPose() - Autonomous Movement
```java
public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
        3.0,  // Max velocity (m/s)
        3.0,  // Max acceleration (m/s²)
        2*Math.PI,  // Max angular velocity (rad/s)
        4*Math.PI   // Max angular acceleration (rad/s²)
    );
    
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0  // Goal end velocity
    );
}
```

**Uses PathPlanner to automatically navigate to a pose.**

---

### Shooter Subsystem
**Location:** `src/main/java/frc/robot/Subsystems/Shooter.java`

**Purpose:** Dual flywheel mechanism for launching game pieces

```java
public class Shooter extends CSubsystem {
    // ═══════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════
    public SparkFlex shooter_motor_controller = 
        new SparkFlex(10, MotorType.kBrushless);
    
    // ═══════════════════════════════════════════════════
    // YAMS CONFIGURATION (PID, Feedforward, Limits)
    // ═══════════════════════════════════════════════════
    public SmartMotorControllerConfig smc_config = 
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                SHOOTER_KP,    // Proportional gain
                SHOOTER_KI,    // Integral gain
                SHOOTER_KD,    // Derivative gain
                DegreesPerSecond.of(90),         // Max velocity
                DegreesPerSecondPerSecond.of(45) // Max acceleration
            )
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
            .withGearing(1)  // Direct drive (1:1)
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40));
    
    // ═══════════════════════════════════════════════════
    // YAMS WRAPPER (Abstracts REV API)
    // ═══════════════════════════════════════════════════
    public SmartMotorController smc = new SparkWrapper(
        shooter_motor_controller,    // Physical controller
        DCMotor.getNeoVortex(1),     // Motor physics model
        smc_config                   // Configuration
    );
    
    // ═══════════════════════════════════════════════════
    // FLYWHEEL MECHANISM (High-level control)
    // ═══════════════════════════════════════════════════
    public FlyWheelConfig shooter_config = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))   // Wheel diameter
        .withMass(Pounds.of(1));      // Flywheel mass
    
    public FlyWheel shooter_controller = new FlyWheel(shooter_config);
    
    // ═══════════════════════════════════════════════════
    // STATE TRACKING
    // ═══════════════════════════════════════════════════
    private ShooterState state = ShooterState.Off;
    
    // ═══════════════════════════════════════════════════
    // SYSID (System Identification for tuning)
    // ═══════════════════════════════════════════════════
    private final SysIdRoutine sysIdRoutine;
    
    public Shooter() {
        // Setup SysId characterization routines
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> smc.setVoltage(volts),
                null,
                this
            )
        );
        
        // Register SysId commands to SmartDashboard
        SmartDashboard.putData("Shooter/SysId Quasistatic Forward", 
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Shooter/SysId Dynamic Forward", 
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    }
}
```

**YAMS Architecture Layers:**

```
Your Code
    ↓
FlyWheel (High-level mechanism control)
    ↓
SmartMotorController (Unified interface)
    ↓
SparkWrapper (REV adapter)
    ↓
SparkFlex (Physical motor controller)
    ↓
NEO Vortex Motor
```

**Command Examples:**

```java
/**
 * Shoot at full speed
 */
public CCommand Shoot() {
    return cCommand("StartShooting")
        .onExecute(() -> {
            state = ShooterState.On;
            shooter_controller.setSpeed(
                RobotConstants.ShooterSubsystemConstants.forwardsOnSpeeds
            );
        })
        .onEnd(() -> {
            state = ShooterState.Off;
            shooter_motor_controller.setVoltage(0);
        });
}

/**
 * Manual voltage control (for testing)
 */
public CCommand ManualSpinForward() {
    return cCommand("ManualSpinForward")
        .onExecute(() -> {
            state = ShooterState.ManualForward;
            shooter_motor_controller.setVoltage(6.0);
        })
        .onEnd(() -> {
            state = ShooterState.Off;
            shooter_motor_controller.setVoltage(0);
        });
}
```

**Periodic Methods:**

```java
@Override
public void periodic() {
    // Update telemetry every 20ms (real robot)
    shooter_controller.updateTelemetry();
}

@Override
public void simulationPeriodic() {
    // Update physics simulation every 20ms
    shooter_controller.simIterate();
}
```

---

### Intake Subsystem
**Location:** `src/main/java/frc/robot/Subsystems/Intake.java`

**Purpose:** Collects game pieces from the field

```java
public class Intake extends CSubsystem {
    private final SparkMax intakeMotorController = 
        new SparkMax(13, MotorType.kBrushless);
    
    private final SmartMotorControllerConfig smcConfig = 
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(0.001, 0, 0, 
                DegreesPerSecond.of(90), 
                DegreesPerSecondPerSecond.of(45))
            .withGearing(new MechanismGearing(
                GearBox.fromReductionStages(4, 4)  // 16:1 reduction
            ))
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40));
    
    private final SmartMotorController intakeController = 
        new SparkWrapper(
            intakeMotorController, 
            DCMotor.getNeoVortex(1), 
            smcConfig
        );
    
    private IntakeState state = IntakeState.OFF;
    
    /**
     * Turn intake on (collect game pieces)
     */
    public CCommand SetIntakeOn() {
        return cCommand().onInitialize(() -> {
            this.intakeController.setVoltage(Volts.of(9));
        });
    }
    
    /**
     * Turn intake off
     */
    public CCommand SetIntakeOff() {
        return cCommand().onInitialize(() -> {
            this.intakeController.setVoltage(Volts.of(0));
        });
    }
}
```

**Gearing Example:**
```java
.withGearing(new MechanismGearing(
    GearBox.fromReductionStages(4, 4)
))
// This creates a 16:1 reduction (4:1 × 4:1)
// Motor spins 16 times for 1 output rotation
```

---

### Indexer Subsystem
**Location:** `src/main/java/frc/robot/Subsystems/Indexer.java`

**Purpose:** Transfers game pieces between intake and shooter

```java
public class Indexer extends CSubsystem {
    private final SparkMax indexerMotorController = 
        new SparkMax(15, MotorType.kBrushless);
    
    private final SmartMotorController indexerController = 
        new SparkWrapper(
            indexerMotorController,
            DCMotor.getNeoVortex(1),
            smcConfig
        );
    
    // ═══════════════════════════════════════════════════
    // SUBSYSTEM DEPENDENCIES
    // ═══════════════════════════════════════════════════
    private Shooter shooterSubsystem;
    private Intake intakeSubsystem;
    
    private IndexerState indexerState = IndexerState.OFF;
    
    /**
     * Constructor - requires shooter and intake references
     */
    public Indexer(Shooter shooterSubsystem, Intake intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        
        // Setup SysId...
    }
    
    /**
     * Smart indexing - automatically responds to subsystem states
     */
    public CCommand indexerHandler() {
        return cCommand("IndexerHandler")
            .onExecute(() -> {
                // If shooter is on, feed it
                if (shooterSubsystem.getState() == ShooterState.On) {
                    indexerState = IndexerState.SHOOTER;
                    indexerController.setVelocity(
                        IndexerSubsystemConstants.forwardsOnSpeeds
                    );
                }
                // If intake is on, receive game pieces
                else if (intakeSubsystem.getState() == IntakeState.ON) {
                    indexerState = IndexerState.HOPPER;
                    indexerController.setVelocity(
                        IndexerSubsystemConstants.backwardsOnSpeeds
                    );
                }
                // Otherwise, stop
                else {
                    indexerState = IndexerState.OFF;
                    indexerController.setVelocity(RPM.of(0));
                }
            });
    }
    
    /**
     * Manual control commands
     */
    public CCommand manualForward() {
        return cCommand("ManualForward")
            .onInitialize(() -> {
                this.indexerMotorController.setVoltage(9);
            })
            .onEnd(() -> {
                this.indexerController.setVelocity(RPM.of(0));
            });
    }
}
```

**Key Concept:** Indexer **coordinates** with other subsystems by reading their states.

---

### Vision Subsystem
**Location:** `src/main/java/frc/robot/Subsystems/Vision/`

**Architecture:**

```
Vision (Interface)
    ↓
├── RealVision (Physical Limelight cameras)
└── SimVision (Simulated vision - not yet implemented)
```

**Vision Interface:**
```java
public interface Vision {
    /** Check if AprilTag is visible */
    boolean seesAprilTag();
    
    /** Change vision pipeline */
    void changeFilter(int id);
    
    /** Get detected AprilTag ID */
    Optional<Integer> getTagID();
    
    /** Get horizontal offset (tx) */
    Optional<Double> getTx();
    
    /** Get vertical offset (ty) */
    Optional<Double> getTy();
    
    /** Get target area (ta) */
    Optional<Double> getTa();
}
```

**RealVision Implementation:**
```java
public class RealVision implements Vision, Subsystem {
    private final Drive driveSubsystem;
    private final Limelight back_limelight;
    private final Limelight front_limelight;
    
    public RealVision(Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.front_limelight = new Limelight("front_limelight");
        this.back_limelight = new Limelight("back_limelight");
    }
    
    public boolean seesAprilTag() {
        Optional<LimelightResults> front_results = 
            front_limelight.getLatestResults();
        
        if (front_results.isPresent()) {
            LimelightResults data = front_results.get();
            return data.targets_Fiducials != null && 
                   data.targets_Fiducials.length > 0;
        }
        
        // Check back camera if front doesn't see anything
        Optional<LimelightResults> back_results = 
            back_limelight.getLatestResults();
        
        if (back_results.isPresent()) {
            LimelightResults data = back_results.get();
            return data.targets_Fiducials != null && 
                   data.targets_Fiducials.length > 0;
        }
        
        return false;
    }
    
    @Override
    public void periodic() {
        // Update robot pose from vision every 20ms
        Optional<LimelightResults> front_results = 
            front_limelight.getLatestResults();
        
        if (front_results.isPresent()) {
            Pose2d currentPose = front_results.get().getBotPose2d();
            this.driveSubsystem.updatePose(currentPose);
        } else {
            // Try back camera
            Optional<LimelightResults> back_results = 
                back_limelight.getLatestResults();
            
            if (back_results.isPresent()) {
                Pose2d currentPose = back_results.get().getBotPose2d();
                this.driveSubsystem.updatePose(currentPose);
            }
        }
    }
}
```

**Vision Flow:**
1. Limelight detects AprilTag
2. Calculates robot pose from tag
3. Sends pose to Drive subsystem
4. Drive fuses vision with odometry

---

## Custom Framework

### CSubsystem - Custom Subsystem Base
**Location:** `src/main/java/frc/robot/Libs/CSubsystem.java`

```java
public class CSubsystem extends SubsystemBase {
    public CSubsystem() {
        super();
    }
    
    /**
     * Creates a command that requires this subsystem
     */
    public CCommand cCommand() {
        return new CCommand(this);
    }
    
    /**
     * Creates a named command that requires this subsystem
     */
    public CCommand cCommand(String name) {
        return new CCommand(name, this);
    }
}
```

**Benefits:**
- Shorter syntax: `cCommand()` instead of `new Command()`
- Automatically requires the subsystem
- Cleaner code

**Example:**
```java
// Traditional WPILib
return new FunctionalCommand(
    () -> {},           // initialize
    () -> {},           // execute
    interrupted -> {},  // end
    () -> false,        // isFinished
    this                // requirements
);

// With CCommand
return cCommand()
    .onInitialize(() -> {})
    .onExecute(() -> {})
    .onEnd(() -> {})
    .until(() -> false);
```

---

### CCommand - Command Builder
**Location:** `src/main/java/frc/robot/Libs/CCommand.java`

```java
public class CCommand extends Command {
    private Runnable onInitialize = () -> {};
    private Runnable onExecute = () -> {};
    private BooleanConsumer onEnd = interrupted -> {};
    private BooleanSupplier isFinished = () -> false;
    
    public CCommand(String name, Subsystem... requirements) {
        addRequirements(requirements);
        setName(name);
    }
    
    /**
     * Set the initialize method
     */
    public CCommand onInitialize(Runnable onInitialize) {
        this.onInitialize = onInitialize;
        return this;
    }
    
    /**
     * Set the execute method (runs every 20ms)
     */
    public CCommand onExecute(Runnable onExecute) {
        this.onExecute = onExecute;
        return this;
    }
    
    /**
     * Set the end method
     */
    public CCommand onEnd(BooleanConsumer onEnd) {
        this.onEnd = onEnd;
        return this;
    }
    
    /**
     * Set completion condition
     */
    public CCommand until(BooleanSupplier condition) {
        this.isFinished = condition;
        return this;
    }
    
    // Lifecycle methods called by scheduler
    @Override
    public void initialize() {
        onInitialize.run();
    }
    
    @Override
    public void execute() {
        onExecute.run();
    }
    
    @Override
    public void end(boolean interrupted) {
        onEnd.accept(interrupted);
    }
    
    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}
```

**Builder Pattern:**
```java
return cCommand("MyCommand")
    .onInitialize(() -> {
        // Setup code (runs once)
    })
    .onExecute(() -> {
        // Main code (runs every 20ms)
    })
    .onEnd(interrupted -> {
        // Cleanup code (runs once)
    })
    .until(() -> {
        // Completion condition
        return sensor.getValue() > 10;
    });
```

---

## Configuration System

### RobotConstants.java
**Location:** `src/main/java/frc/robot/Constants/RobotConstants.java`

**Nested Classes for Organization:**

```java
public class RobotConstants {
    /**
     * Controller deadbands
     */
    public static final class ControllerConstants {
        public static final double deadbandX = 0.15;
        public static final double deadbandY = 0.15;
    }
    
    /**
     * Swerve drive speeds and PID
     */
    public static final class DriveSubsystemConstants {
        public static final double maxTurnSpeed = 90;  // degrees/sec
        public static final double maxSpeed = 6;  // m/s
        
        public static final PIDConstants translationPidConstants = 
            new PIDConstants(10, 0, 0);
        public static final PIDConstants rotationPidConstants = 
            new PIDConstants(8, 0, 0);
    }
    
    /**
     * Shooter flywheel speeds and PID
     */
    public static final class ShooterSubsystemConstants {
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        public static final AngularVelocity backwardsOnSpeeds = RPM.of(-100);
        public static final AngularVelocity manualSpinSpeed = RPM.of(20);
        
        // PID Constants
        public static final double SHOOTER_KP = 50;
        public static final double SHOOTER_KI = 0.0;
        public static final double SHOOTER_KD = 0.0;
    }
    
    /**
     * Intake speeds
     */
    public static final class IntakeSubsystemConstants {
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
    }
    
    /**
     * Indexer speeds
     */
    public static final class IndexerSubsystemConstants {
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        public static final AngularVelocity backwardsOnSpeeds = RPM.of(100);
        public static final AngularVelocity manualSpeed = RPM.of(1000);
    }
}
```

**Usage:**
```java
// In code:
shooter_controller.setSpeed(
    RobotConstants.ShooterSubsystemConstants.forwardsOnSpeeds
);
```

---

### State Enums
**Location:** `src/main/java/frc/robot/Constants/`

**ShooterState.java:**
```java
public enum ShooterState {
    On,
    Off,
    Rev,
    Backwards,
    ManualForward,
    ManualBackward
}
```

**IntakeState.java:**
```java
public enum IntakeState {
    ON,
    REV,
    OFF
}
```

**IndexerState.java:**
```java
public enum IndexerState {
    SHOOTER,
    HOPPER,
    OFF,
    MANUAL_FORWARD,
    MANUAL_BACKWARD
}
```

**Usage in State Machines:**
```java
private ShooterState state = ShooterState.Off;
private ShooterState currentState = ShooterState.Off;

public CCommand shooterHandler() {
    return cCommand().onExecute(() -> {
        if (state != currentState) {
            switch (state) {
                case Off:
                    currentState = ShooterState.Off;
                    shooter_controller.setSpeed(RPM.of(0));
                    break;
                case On:
                    currentState = ShooterState.On;
                    shooter_controller.setSpeed(RPM.of(3000));
                    break;
            }
        }
    });
}
```

---

### JSON Configuration Files
**Location:** `src/main/deploy/swerve/`

**swervedrive.json:**
```json
{
  "maxSpeed": 4.5,
  "optimalVoltage": 12,
  "imu": {
    "type": "pigeon2",
    "id": 0,
    "canbus": "canivore"
  },
  "modules": [
    "frontleft.json",
    "frontright.json",
    "backleft.json",
    "backright.json"
  ]
}
```

**frontleft.json (Module Config):**
```json
{
  "drive": {
    "type": "sparkflex",
    "id": 1,
    "canbus": null
  },
  "angle": {
    "type": "sparkflex",
    "id": 2,
    "canbus": null
  },
  "encoder": {
    "type": "cancoder",
    "id": 9,
    "canbus": "canivore"
  },
  "inverted": {
    "drive": false,
    "angle": false
  },
  "absoluteEncoderOffset": -45.0,
  "location": {
    "x": 12,
    "y": 12
  }
}
```

**Loaded by YAGSL:**
```java
File swerveJsonDirectory = new File(
    Filesystem.getDeployDirectory(), 
    "swerve"
);
swerveDrive = new SwerveParser(swerveJsonDirectory)
    .createSwerveDrive(maxSpeed, startingPose);
```

---

## Code Flow Examples

### Example 1: Button Press to Motor Spin

**User Action:** Driver presses A button

```
1. HARDWARE LEVEL
   └── Xbox Controller sends signal to Driver Station
   └── Driver Station sends signal to RoboRIO

2. ROBOTCONTAINER LEVEL (Button Binding)
   └── configureBindings() has:
       driverController.a().whileTrue(shooter_subsystem.Shoot())

3. TRIGGER FIRES
   └── Trigger detects button state = pressed
   └── Calls shooter_subsystem.Shoot()
   └── Returns a CCommand object

4. COMMAND SCHEDULING
   └── CommandScheduler.schedule(command)
   └── Command added to active commands list

5. ROBOTPERIODIC (Every 20ms)
   └── CommandScheduler.run()
   └── For each active command:
       ├── First cycle: Call initialize()
       ├── Every cycle: Call execute()
       └── Check isFinished()

6. COMMAND EXECUTION
   └── Shoot().onExecute() runs:
       state = ShooterState.On;
       shooter_controller.setSpeed(RPM.of(3000));

7. YAMS LAYER
   └── FlyWheel.setSpeed(3000)
   └── SmartMotorController calculates PID + Feedforward
   └── SparkWrapper translates to REV API

8. HARDWARE LAYER
   └── SparkFlex.setVoltage(calculatedVoltage)
   └── CAN message sent to motor controller
   └── Motor controller drives NEO Vortex motor
   └── Motor spins!

9. TELEMETRY (Every 20ms)
   └── periodic() calls shooter_controller.updateTelemetry()
   └── Data published to SmartDashboard

10. BUTTON RELEASE
    └── Trigger detects button state = released
    └── Command.cancel() called
    └── onEnd() executes:
        shooter_motor_controller.setVoltage(0);
    └── Motor stops
```

---

### Example 2: Autonomous Path Following

**User Action:** Match starts in autonomous mode

```
1. AUTONOMOUSINIT
   └── autonomousCommand = robotContainer.getAutonomousCommand()
   └── Returns PathPlanner auto from Autos class

2. AUTO COMMAND STRUCTURE
   └── PathPlannerAuto contains:
       ├── Path1 (drive to position)
       ├── Shoot command
       ├── Path2 (drive to next position)
       └── Intake command

3. PATH EXECUTION
   └── AutoBuilder.followPath(path1)
   └── Every 20ms:
       ├── Calculate current position (odometry + vision)
       ├── Calculate trajectory setpoint
       ├── Calculate drive velocities (PID)
       └── Drive swerve modules

4. VISION INTEGRATION
   └── Vision.periodic() runs every 20ms:
       ├── Limelight detects AprilTag
       ├── Calculates robot pose
       └── Calls driveSubsystem.updatePose(visionPose)
   
   └── Drive.updatePose():
       └── swerveDrive.addVisionMeasurement(pose, timestamp)
       └── Kalman filter fuses vision with odometry
       └── More accurate position estimate

5. COMMAND TRANSITIONS
   └── Path1 completes (isFinished = true)
   └── Sequential command moves to next
   └── Shoot command starts
   └── Shooter spins up
   └── Path2 starts...
```

---

### Example 3: Smart Indexer Coordination

**Scenario:** Intake turns on, indexer should automatically respond

```
1. INTAKE COMMAND
   └── Button pressed: intake_subsystem.SetIntakeOn()
   └── onInitialize runs:
       state = IntakeState.ON;
       intakeController.setVoltage(Volts.of(9));
   └── Intake motor spins

2. INDEXER DEFAULT COMMAND
   └── indexerHandler() runs every 20ms
   └── onExecute checks:
       if (intakeSubsystem.getState() == IntakeState.ON) {
           // Automatically run indexer towards hopper
           indexerController.setVelocity(backwardsOnSpeeds);
       }

3. COORDINATION ACHIEVED
   └── No explicit communication needed
   └── Indexer reads intake state
   └── Responds automatically
   └── When intake stops, indexer stops

4. PRIORITY OVERRIDE
   └── If shooter turns on while intake running:
       if (shooterSubsystem.getState() == ShooterState.On) {
           // Shooter has priority
           indexerController.setVelocity(forwardsOnSpeeds);
       }
```

---

## Best Practices

### 1. Configuration Before Operation

```java
// ✅ CORRECT - Configure in constructor
public Shooter() {
    // Create motor
    SparkFlex motor = new SparkFlex(10, MotorType.kBrushless);
    
    // Configure before use
    SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(kP, kI, kD, maxVel, maxAccel);
    
    // Wrap motor
    SmartMotorController controller = new SparkWrapper(motor, dcMotor, config);
}

// ❌ WRONG - Configure during operation
public CCommand Shoot() {
    return cCommand().onExecute(() -> {
        motor.configure(config);  // DON'T DO THIS
        motor.setVoltage(6);
    });
}
```

---

### 2. Use Default Commands

```java
// ✅ CORRECT - Default command always running
public Shooter() {
    setDefaultCommand(shooterHandler());  // Runs in background
}

public CCommand shooterHandler() {
    return cCommand().onExecute(() -> {
        // State machine runs every 20ms
        if (state != currentState) {
            switch (state) { /* ... */ }
        }
    });
}

// ❌ WRONG - No default command
public Shooter() {
    // No default command = subsystem idle
}
```

---

### 3. Telemetry in periodic()

```java
// ✅ CORRECT - Update telemetry every cycle
@Override
public void periodic() {
    shooter_controller.updateTelemetry();
    SmartDashboard.putNumber("Shooter/State", state.ordinal());
}

// ✅ CORRECT - Simulation physics
@Override
public void simulationPeriodic() {
    shooter_controller.simIterate();
}

// ❌ WRONG - Update in commands
public CCommand Shoot() {
    return cCommand().onExecute(() -> {
        shooter_controller.updateTelemetry();  // DON'T DO THIS
    });
}
```

---

### 4. Subsystem Dependencies

```java
// ✅ CORRECT - Pass dependencies through constructor
public Indexer(Shooter shooterSubsystem, Intake intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
}

// In RobotContainer:
shooter = new Shooter();
intake = new Intake();
indexer = new Indexer(shooter, intake);  // Clear dependencies

// ❌ WRONG - Static references or singletons
public class Indexer {
    public void checkState() {
        if (GlobalRobot.shooter.getState() == On) { /* ... */ }
    }
}
```

---

### 5. State Management

```java
// ✅ CORRECT - Commands change state, handler executes
private ShooterState targetState = ShooterState.Off;
private ShooterState currentState = ShooterState.Off;

public CCommand Shoot() {
    return cCommand().onInitialize(() -> {
        targetState = ShooterState.On;  // Request state change
    });
}

public CCommand shooterHandler() {
    return cCommand().onExecute(() -> {
        if (targetState != currentState) {
            // Execute state transition
            switch (targetState) { /* ... */ }
        }
    });
}

// ❌ WRONG - Direct motor control in multiple places
public CCommand Shoot() {
    return cCommand().onExecute(() -> {
        motor.setVoltage(6);  // Bypasses state machine
    });
}
```

---

### 6. Command Composition

```java
// ✅ CORRECT - Build complex commands from simple ones
public Command autoShootSequence() {
    return Commands.sequence(
        intake.SetIntakeOn(),
        Commands.waitSeconds(2),
        Commands.parallel(
            shooter.Shoot(),
            indexer.shooterIndexing()
        ),
        Commands.waitSeconds(1),
        shooter.StopShooting(),
        intake.SetIntakeOff()
    );
}

// ✅ CORRECT - Use decorators
public Command shootWithTimeout() {
    return shooter.Shoot()
        .withTimeout(5.0)
        .alongWith(indexer.shooterIndexing());
}
```

---

### 7. Error Handling

```java
// ✅ CORRECT - Graceful error handling
public Drive() {
    File swerveDir = new File(Filesystem.getDeployDirectory(), "swerve");
    
    try {
        swerveDrive = new SwerveParser(swerveDir)
            .createSwerveDrive(maxSpeed, startingPose);
    } catch (IOException e) {
        DriverStation.reportError(
            "Failed to load swerve config: " + e.getMessage(), 
            e.getStackTrace()
        );
        throw new RuntimeException(e);
    }
}

// ✅ CORRECT - Optional for vision data
public Optional<Integer> getTagID() {
    Optional<LimelightResults> results = limelight.getLatestResults();
    
    if (results.isPresent() && seesAprilTag()) {
        AprilTagFiducial[] tags = results.get().targets_Fiducials;
        if (tags.length > 0) {
            return Optional.of((int) tags[0].fiducialID);
        }
    }
    
    return Optional.empty();  // No tag found
}
```

---

## Summary

### Project Structure
```
Robot.java              ← Entry point, lifecycle manager
RobotContainer.java     ← Central hub, creates everything
Constants/              ← All configuration values
├── RobotConstants.java
├── ShooterState.java
├── IntakeState.java
└── IndexerState.java

Subsystems/             ← Hardware control
├── Drive.java          ← Swerve drive + vision
├── Shooter.java        ← Flywheel mechanism
├── Intake.java         ← Game piece collection
├── Indexer.java        ← Transfer mechanism
└── Vision/
    ├── Vision.java     ← Interface
    ├── RealVision.java ← Limelight implementation
    └── SimVision.java  ← Simulation (not implemented)

Libs/                   ← Custom framework
├── CCommand.java       ← Command builder
├── CSubsystem.java     ← Subsystem base class
└── LimelightHelpers.java

Autos.java              ← Autonomous management
```

### Key Concepts

1. **Command-Based Architecture** - Commands control subsystems
2. **State Machines** - Enums + handlers for complex behavior
3. **YAMS Abstraction** - Hardware-independent motor control
4. **Vision Fusion** - AprilTags improve odometry
5. **PathPlanner Integration** - Complex autonomous paths
6. **SysId Ready** - Characterization for PID/FF tuning

### Development Workflow

1. **Configure Constants** - Set speeds, PID values
2. **Build Subsystems** - Motors, sensors, state machines
3. **Create Commands** - Actions the robot can perform
4. **Bind Buttons** - Map controllers to commands
5. **Test & Tune** - Use SysId, telemetry, simulation
6. **Build Autos** - PathPlanner paths and command sequences

---

*This guide covers the architecture of the 2026 FRC robot project. For lifecycle and program flow details, see `FRC_PROGRAMMING_GUIDE.md`.*
