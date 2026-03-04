// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeState;
import frc.robot.Libs.FuelSim;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shaker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Vision.Vision;
import swervelib.parser.json.modules.DriveConversionFactorsJson;

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
      // Provide shooter with drive reference so sim spawning can be gated to autodrive target
      shooter_subsystem.setDriveSubsystem(drive_subsystem);
      intake_subsystem = new Intake(shooter_subsystem, drive_subsystem);
      // Wire intake into shooter so shooter can request ball removal from the hopper
      shooter_subsystem.setIntakeSubsystem(intake_subsystem);
    indexer_subsystem = new Indexer( shooter_subsystem, intake_subsystem, drive_subsystem );
    climber_subsystem = new Climber();
    shaker_subsystem = new Shaker(indexer_subsystem);

    autos = new Autos(this, drive_subsystem );

    configureBindings();
    configureAuto();
    if ( Robot.isSimulation() ) {
      configureSim();
    }
  }

  private void configureAuto() {
    autos.addCommand("Shoot", shooter_subsystem.Shoot());
    autos.addCommand("Intake", intake_subsystem.IntakeOn());
    autos.addCommand("ClimberUp", climber_subsystem.climbUp());
    autos.addCommand("ClimberZero", climber_subsystem.climbZero());
    autos.addCommand("ClimberDown", climber_subsystem.climb());
  }

  private void configureSim() {
    fuelSim.registerRobot(
        Inches.of(27.5).in(Meters), // from left to right in meters
        Inches.of(27.5).in(Meters), // from front to back in meters
        Inches.of(5).in(Meters), // from floor to top of bumpers in meters
        drive_subsystem::getPose, // Supplier<Pose2d> of robot pose
        drive_subsystem.getSwerveDrive()::getFieldVelocity); // Supplier<ChassisSpeeds> of field-centric chassis speeds

    fuelSim.registerIntake(
      Inches.of(20.5/2),
      Inches.of(27.5+2),
      Inches.of(-27.5/3), 
      Inches.of(27.5/3),
      () -> {
        return intake_subsystem.getState() == IntakeState.ON && !intake_subsystem.isHopperFull();
      }, 
      () -> {
        intake_subsystem.addToHopper();
      });   

    fuelSim.spawnStartingFuel();
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

    driverController.povUp().whileTrue(new DeferredCommand(drive_subsystem::driveThroughTrenchSS, Set.of(drive_subsystem)));
    driverController.povDown().whileTrue(new DeferredCommand(drive_subsystem::driveThroughTrenchOS, Set.of(drive_subsystem)));
    
    drive_subsystem.setDefaultCommand(
      drive_subsystem.driveWithChassisSpeedsSupplier(
        drive_subsystem.buildTrenchStream()
      ));

    driverController.a().whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToElevator()));

    driverController.leftBumper().and(drive_subsystem::isOnOurSide).whileTrue(
      new ParallelCommandGroup(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToCurveStream()),
        shooter_subsystem.Shoot()
      ));
    // Keep Drive informed when the drive-to-curve input stream is active so other subsystems
    // can gate behavior (indexer/intake) based on whether we're at the pose.
    driverController.leftBumper().onTrue(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(true)));
    driverController.leftBumper().onFalse(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(false)));
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
    // driverController.rightTrigger()
    //   .whileTrue(
    //     drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildHalfDriveStream())
    //   );

    // Drives to a point on the curve (the arc where we are best equipped to shoot accurately)
    // driverController.a()
    //   .whileTrue(
    //     drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToCurveStream())
    //   );

    // Barf
    driverController.rightBumper().whileTrue(intake_subsystem.IntakeBarf());

    // Toggle the intake on / off
    driverController.x().whileTrue(intake_subsystem.intakeToggler());

    driverController.povLeft().whileTrue(new SequentialCommandGroup(
      new DeferredCommand( drive_subsystem::driveThroughTrenchSS, Set.of(drive_subsystem)),
      new InstantCommand(() -> drive_subsystem.setDriveToPoseActive(true)),
      new DeferredCommand(() -> drive_subsystem.driveToTargetPose(drive_subsystem.getClosestPointOnCurve(0), 0), Set.of(drive_subsystem)),
      new ParallelDeadlineGroup(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToCurveStream()),
        shooter_subsystem.Shoot()
      ).withTimeout(10.0),
      new InstantCommand(() -> drive_subsystem.setDriveToPoseActive(false)),
      new DeferredCommand( drive_subsystem::driveThroughTrenchSS, Set.of(drive_subsystem))
    ));
    
    //Co Driver Controls


    // right trigger - shoot forwards, but only when robot is on our side of the field
    // Inform Drive when aim mode is active so indexer/intake can require being aimed before feeding
    driverController.rightTrigger()
      .and(() -> drive_subsystem.isOnOurSide())
      .onTrue(Commands.runOnce(() -> drive_subsystem.setAimModeActive(true)));
    driverController.rightTrigger()
      .and(() -> drive_subsystem.isOnOurSide())
      .onFalse(Commands.runOnce(() -> drive_subsystem.setAimModeActive(false)));

    driverController.rightTrigger()
      .and(() -> drive_subsystem.isOnOurSide())
      .whileTrue(new ParallelCommandGroup(shooter_subsystem.Shoot(), drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildAimingStream())));

    driverController.rightTrigger()
      .and(() -> !drive_subsystem.isOnOurSide())
      .whileTrue(shooter_subsystem.Shoot());

    codriverController.a().whileTrue(climber_subsystem.manualClimbUpVoltage());
    codriverController.b().whileTrue(climber_subsystem.manualClimbDownVoltage());

    // Climber controls on codriver X, Y, and B
    codriverController.povUp().whileTrue(climber_subsystem.climbUp());
    
    // Y button - climb to climbing position (middle position)
    codriverController.povLeft().whileTrue(climber_subsystem.climb());
    
    // B button - climb down
    codriverController.povDown().whileTrue(climber_subsystem.climbZero());
  }

  public Command getAutonomousCommand() {
    return this.autos.getAutonomousCommand();
  }

  public Vision getVision() {
    return this.vision_subsystem;
  }

}
