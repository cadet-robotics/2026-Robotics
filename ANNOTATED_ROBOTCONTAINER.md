# Annotated RobotContainer (walkthrough)

Path: `src/main/java/frc/robot/RobotContainer.java`

This file explains where the subsystems, controllers, and command bindings are created and wired in this project. I've included the relevant snippets and short notes on what to change if you want to add or rewire behavior.

## Purpose

`RobotContainer` is the single place that:
- Instantiates subsystems
- Creates controllers (joysticks/gamepads)
- Configures button bindings
- Sets default commands (e.g., drivetrain teleop)

## Top-level fields

- `private final Autos autos;`
  - The autos helper that likely registers autonomous sequences. It gets the container and drive subsystem in the constructor.

- `private final Drive drive_subsystem;`
  - The drivetrain/swerve subsystem instance used throughout; default commands and drive methods are provided by this class.

- (commented-out) `Vision`, `Intake`, `Indexer`, `Shooter`
  - These are present but commented out. To use them, uncomment and initialize them in the constructor and wire them into commands and binds.

- `driverController` and `codriverController` (both `CommandXboxController`)
  - Controller indices are `0` and `1` respectively. Replace indices if your controllers are mapped differently.

## Constructor overview

Key actions in the constructor:

- Initialize subsystems:

  this.drive_subsystem = new Drive();

  // Other subsystems are present but commented out. Example (commented):
  // this.shooter_subsystem = new Shooter();

- Create `Autos` instance:

  this.autos = new Autos(this, this.drive_subsystem );

  // `Autos` likely needs the RobotContainer to access commands or chooser registration.

- Initialize controllers and run configuration helpers:

  this.driverController = new CommandXboxController(0);
  this.codriverController = new CommandXboxController(1);
  this.configureBindings();
  this.configureDriving();

## Button bindings (`configureBindings`) 

Relevant snippet (from file):

  //this.driverController.a().whileTrue( this.drive_subsystem.dummyDrivePose() );

Actual active binding in the repository:

  this.driverController.a().whileTrue( this.drive_subsystem.dummyDrivePose() );

Explanation:
- The A button on the driver controller runs `dummyDrivePose()` on `drive_subsystem` while the button is held. `dummyDrivePose()` should return a Command (or CommandBase) and is invoked repeatedly by the command scheduler while the button is pressed.

To add more bindings:
- Use the pattern `this.driverController.x().onTrue(...)` or `.whileTrue(...)` for hold behaviors.
- You can bind to co-driver via `this.codriverController` similarly.

## Driving setup (`configureDriving`)

This method sets up deadbanded suppliers for translation and heading and sets the drivetrain default command.

Key parts:

  DoubleSupplier getTranslationX = () -> MathUtil.applyDeadband(this.driverController.getLeftY(), ControllerConstants.deadbandX);
  DoubleSupplier getTranslationY = () -> MathUtil.applyDeadband(this.driverController.getLeftX(), ControllerConstants.deadbandY);
  DoubleSupplier getHeadingX = () -> -1 * this.driverController.getRightX();
  DoubleSupplier getHeadingY = () -> -1 * this.driverController.getRightY();

  Command defaultDrive = drive_subsystem.driveCommand(
      getTranslationX,
      getTranslationY,
      getHeadingX,
      getHeadingY
  );

  this.drive_subsystem.setDefaultCommand(defaultDrive);

Explanation and how to adjust:
- The code uses the left stick for translation and the right stick for heading/rotation.
- Deadbands from `ControllerConstants` are applied to translation axes.
- `drive_subsystem.driveCommand(...)` is a factory method that returns the command implementing teleop driving. To change axis mapping or add modifiers (e.g., slow mode), alter the suppliers or wrap with transforms.

## Autonomous command (`getAutonomousCommand`)

Current implementation returns the drivetrain's default command:

  public Command getAutonomousCommand() {
    return this.drive_subsystem.getDefaultCommand();
  }

Notes:
- This is a placeholder. Normally, you'd return a pre-built autonomous command from `Autos` (or a chooser) here. For example:

  // return autos.someAutoCommand();

- Check `Autos.java` for available auto sequences and register them with `SendableChooser` if desired.

## How to wire in the other subsystems (practical steps)

1. Uncomment the subsystem field declarations at the top (Vision, Intake, Indexer, Shooter).
2. Initialize them in the constructor similarly to `drive_subsystem`.
3. Create commands (or use existing ones) that act on the subsystems.
4. Bind those commands to controller buttons in `configureBindings()`.
5. If a subsystem needs a default command, set it with `setDefaultCommand(...)`.

## Where to look next in the codebase

- `Drive.java` for how `driveCommand(...)`, `dummyDrivePose()`, and `getDefaultCommand()` are implemented.
- `Autos.java` for autonomous routine registration and implementations.
- `Configs.java` and `Constants.java` for controller deadbands, port mappings, and swerve config parsing.
