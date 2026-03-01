// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Libs.FuelSim;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shaker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Vision.Vision;

public class RobotContainer {

  private final Autos autos;
  private final Drive drive_subsystem;
  private final Vision vision_subsystem;
  private final Intake intake_subsystem;
  private final Indexer indexer_subsystem;
  private final Shooter shooter_subsystem;
  private final Climber climber_subsystem;
  private final Shaker shaker_subsystem;

  public FuelSim fuelSim = new FuelSim("FuelSim");

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController codriverController = new CommandXboxController(1);

  public RobotContainer() {
    // Setup and initialize Subsystems here
    drive_subsystem = new Drive(driverController, codriverController);
    vision_subsystem = drive_subsystem.getVision();
    shooter_subsystem = new Shooter(fuelSim);
    intake_subsystem = new Intake(shooter_subsystem, drive_subsystem);
    indexer_subsystem = new Indexer( shooter_subsystem, intake_subsystem, drive_subsystem );
    climber_subsystem = new Climber();
    shaker_subsystem = new Shaker(indexer_subsystem);

    autos = new Autos(this, drive_subsystem );

    configureBindings();
    if ( Robot.isSimulation() ) {
      configureSim();
    }
  }

  private void configureSim() {
    fuelSim.registerRobot(
        Inches.of(27.5).in(Meters), // from left to right in meters
        Inches.of(27.5).in(Meters), // from front to back in meters
        Inches.of(5).in(Meters), // from floor to top of bumpers in meters
        drive_subsystem::getPose, // Supplier<Pose2d> of robot pose
        drive_subsystem.getSwerveDrive()::getFieldVelocity); // Supplier<ChassisSpeeds> of field-centric chassis speeds

    fuelSim.start(); // enables the simulation to run (updateSim must still be called periodically)
  }

  private void configureBindings() {
    // Driver Controls

    // SysId complete routine for shooter characterization - runs all 4 tests in sequence
    // driverController.a().onTrue(shooter_subsystem.getCompleteSysIdRoutine());

    // Reset odometry to current Limelight pose
    // driverController.b().onTrue(drive_subsystem.resetOdometryWithVision());

    // Reset Gyro
    driverController.b().whileTrue(drive_subsystem.resetOdom());
  
    drive_subsystem.setDefaultCommand(
      drive_subsystem.driveWithChassisSpeedsSupplier(
        drive_subsystem.buildRelativeTurningStream()
      ));

    driverController.leftBumper().whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToCurveStream()));
    // Shoots, Shakes, and Aims
    // driverController.rightTrigger()
    //   .whileTrue( 
    //     new ParallelCommandGroup(
    //       shooter_subsystem.Shoot(),
    //       drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildAimingStream())
    //     ));
    
    // Uses Relative Turning
    driverController.leftTrigger()
      .whileTrue(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDefaultStream())
      );

    // Cuts the speed of the bot in half for more precise maneuvering
    driverController.rightTrigger()
      .whileTrue(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildHalfDriveStream())
      );

    // Drives to a point on the curve (the arc where we are best equipped to shoot accurately)
    // driverController.a()
    //   .whileTrue(
    //     drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToCurveStream())
    //   );

    // Barf
    driverController.rightBumper().whileTrue(intake_subsystem.IntakeBarf());

    // Toggle the intake on / off
    driverController.x().whileTrue(intake_subsystem.intakeToggler());

    
    //Co Driver Controls


    // right trigger - shoot forwards
    codriverController.rightTrigger().whileTrue(shooter_subsystem.Shoot());

    // Automatic driving to the closest climb position
    // codriverController.a().whileTrue(drive_subsystem.driveToClimb());

    codriverController.a().whileTrue(climber_subsystem.manualClimbUpVoltage());
    codriverController.b().whileTrue(climber_subsystem.manualClimbDownVoltage());

    // Climber controls on codriver X, Y, and B
    codriverController.povUp().whileTrue(climber_subsystem.climbUp());
    
    // Y button - climb to climbing position (middle position)
    codriverController.povLeft().whileTrue(climber_subsystem.climb());
    
    // B button - climb down
    codriverController.povDown().whileTrue(climber_subsystem.climbZero());
  }

  // public void configureDriving() {
  //   DoubleSupplier getTranslationX = () -> -1 * MathUtil.applyDeadband(this.driverController.getLeftY(),
  //       ControllerConstants.deadbandX);
  //   DoubleSupplier getTranslationY = () -> -1 * MathUtil.applyDeadband(this.driverController.getLeftX(),
  //       ControllerConstants.deadbandY);
  //   DoubleSupplier getHeadingX = () -> -1 * driverController.getRightX();
  //   DoubleSupplier getHeadingY = () -> -1 * driverController.getRightY();

  //   SwerveInputStream baseStream = SwerveInputStream.of(drive_subsystem.getSwerveDrive(), getTranslationX, getTranslationY)
  //     .scaleTranslation(0.8)
  //     .allianceRelativeControl(true)
  //     .deadband(0.12);

  //   // The default SIS
  //   SwerveInputStream defaultStream = baseStream.copy()
  //     .withControllerHeadingAxis(getHeadingX, getHeadingY)
  //     .headingWhile(true);

  //   // A SIS with relative turning
  //   SwerveInputStream relativeTurning = baseStream.copy()
  //     .withControllerRotationAxis(() -> -1 * driverController.getRightX());

  //   SwerveInputStream halfSpeed = baseStream.copy()
  //     .scaleTranslation(0.5)
  //     .scaleRotation(0.5);

    // A SIS which looks at the hub but uses a heading vector that's opposite the hub direction
    // SwerveInputStream looking = baseStream
    //   .copy()
    //   .aim(FieldConstants.hub.rotateAround(drive_subsystem.getPose().getTranslation(), Rotation2d.k180deg))
    //   .aimWhile(true);

    // A SIS which uses the auto drive to pose to align to a point on the curve
    // SwerveInputStream autoDrive = baseStream
    //   .copy()
    //   .driveToPose(
    //     () -> drive_subsystem.getClosestPointOnCurve(driverController.getLeftX()),
    //     RobotConstants.DriveSubsystemConstants.translationProfiledController, 
    //     RobotConstants.DriveSubsystemConstants.rotationProfiledController
    //   )
    //   .driveToPoseEnabled(true);

    // Will use default by default
  //   drive_subsystem.setDefaultCommand(
  //     drive_subsystem.driveWithChassisSpeedsSupplier(defaultStream).withName("headingDrive")
  //   );

  //   BooleanSupplier relativeDriveCondition = driverController.leftTrigger()::getAsBoolean;
  //   BooleanSupplier aimDriveCondition = driverController.rightTrigger()::getAsBoolean;
  //   BooleanSupplier autoDriveCondition = driverController.a()::getAsBoolean;

  // // We do a little bit of binding here which might be bad idk
  //   driverController
  //     .leftTrigger()
  //     .and(driverController.a().negate())
  //     .and(() -> !autoDriveCondition.getAsBoolean())
  //     .whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(relativeTurning).withName("rotationDrive"));

    // driverController
    //   .rightTrigger()
    //   .whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(halfSpeed).withName("halfDrive"));

    // driverController
    //   .rightTrigger()
    //   .and(() -> {return !autoDriveCondition.getAsBoolean();})
    //   .whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(looking));
    // // Inform Drive when aim mode is active so indexer/intake can require being aimed before feeding
    // driverController.rightTrigger().onTrue(Commands.runOnce(() -> drive_subsystem.setAimModeActive(true)));
    // driverController.rightTrigger().onFalse(Commands.runOnce(() -> drive_subsystem.setAimModeActive(false)));
    
    // driverController
    //   .a()
    //   .whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(autoDrive));
    // // // Keep Drive informed when the drive-to-pose input stream is active so other subsystems
    // // can gate behavior (indexer/intake) based on whether we're at the pose.
    // driverController.a().onTrue(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(true)));
    // driverController.a().onFalse(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(false)));

    // Button to fire a lead shot using SmartDashboard-specified target velocity (for testing)
    // driverController.x().onTrue(
    //   Commands.runOnce(() -> {
    //     double vx = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TargetVx", 0.0);
    //     double vy = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TargetVy", 0.0);
    //     // Subtract robot field velocity so target velocity is relative to the ground minus robot motion.
    //     var speeds = drive_subsystem.getSwerveDrive().getFieldVelocity();
    //     double rvx = speeds.vxMetersPerSecond;
    //     double rvy = speeds.vyMetersPerSecond;
    //     shooter_subsystem.leadAndLaunch(drive_subsystem.getPose(), RobotConstants.FieldConstants.hub, new edu.wpi.first.math.geometry.Translation2d(vx - rvx, vy - rvy));
    //   }, shooter_subsystem)
    // );
  

  public Command getAutonomousCommand() {
    return this.autos.getAutonomousCommand();
  }

  public Vision getVision() {
    return this.vision_subsystem;
  }

}
