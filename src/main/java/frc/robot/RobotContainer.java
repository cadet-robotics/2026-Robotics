// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.RobotConstants;
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
    Trigger manualOverride = new Trigger(() -> Dashboard.getManualOverride());
    Trigger noManualOverride = manualOverride.negate();

    Trigger angleDriveApprovesOfShooting = new Trigger(drive_subsystem::isAimModeActive)
      .and(() -> Dashboard.getManualOverride() 
        || drive_subsystem.isAimedAtHub(Math.toRadians(6))
      );
    
    Trigger curveDriveApprovesOfShooting = new Trigger(drive_subsystem::isDriveToPoseActive)
      .and(() -> {
        if (manualOverride.getAsBoolean()) {
          return true; // If manual override is active, don't gate shooting at all
        }
        // When autodrive is active, allow shooting only if the robot's distance
        // to the hub is within a tolerance of the midRange used to generate the curve.
        double midRange = RobotConstants.ShooterSubsystemConstants.midRange;
        double posTol = midRange * 0.05; // 5% tolerance around midRange
        double hubDist = drive_subsystem.getPose().getTranslation().getDistance(
            RobotConstants.FieldConstants.hub.get().getTranslation());
        boolean distOk = Math.abs(hubDist - midRange) <= posTol;
        boolean angleOk = drive_subsystem.isAimedAtHub(Math.toRadians(6.0));
        return distOk && angleOk;
      });

    Trigger upToSpeedShooting = new Trigger(shooter_subsystem::isUpToSpeed)
      .and(angleDriveApprovesOfShooting)
      .and(curveDriveApprovesOfShooting)
      .or(manualOverride)
      .whileTrue(Commands.parallel(
        indexer_subsystem.IndexerOut(),
        intake_subsystem.IntakeIn()
      ));

    Trigger indexerGoingOut = new Trigger(() -> !indexer_subsystem.IndexerOut().isFinished())
      .whileTrue(shaker_subsystem.Shake());

    Command barf = Commands.parallel(
      intake_subsystem.IntakeBarf(),
      indexer_subsystem.IndexerOut()
    );
    autos.addCommand("Barf", barf);
    

    Command intake = Commands.parallel(
      intake_subsystem.IntakeIn(),
      indexer_subsystem.IndexerIn()
    );
    autos.addCommand("Intake", intake);
    codriverController.x().whileTrue(intake);

    driverController.rightBumper().whileTrue(barf);

    // Reset Gyro
    driverController.b().whileTrue(drive_subsystem.resetOdom());

    // Commands for auto driving through trenches
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

    driverController.leftBumper().onTrue(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(true)));
    driverController.leftBumper().onFalse(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(false)));

    driverController.leftTrigger()
      .whileTrue(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDefaultStream())
      );

    // Cuts the speed of the bot in half for more precise maneuvering
    // driverController.rightTrigger()
    //   .whileTrue(
    //     drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildHalfDriveStream())
    //   );

    driverController.povLeft()
      .and(noManualOverride)
      .whileTrue(new SequentialCommandGroup(
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
      .and(noManualOverride)
      .and(() -> drive_subsystem.isOnOurSide())
      .onTrue(Commands.runOnce(() -> drive_subsystem.setAimModeActive(true)));

    driverController.rightTrigger()
      .and(noManualOverride)
      .and(() -> drive_subsystem.isOnOurSide())
      .onFalse(Commands.runOnce(() -> drive_subsystem.setAimModeActive(false)));

    driverController.rightTrigger()
      .and(() -> drive_subsystem.isOnOurSide())
      .and(noManualOverride)
      .whileTrue(new ParallelCommandGroup(shooter_subsystem.Shoot(), drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildAimingStream())));

    driverController.rightTrigger()
      .and(() -> !drive_subsystem.isOnOurSide())
      .and(noManualOverride)
      .whileTrue(shooter_subsystem.Shoot());

    codriverController.a().whileTrue(climber_subsystem.manualClimbUpVoltage());
    codriverController.b().whileTrue(climber_subsystem.manualClimbDownVoltage());

    codriverController.rightBumper().whileTrue(intake);

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
