// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

  public FuelSim fuelSim;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController codriverController = new CommandXboxController(1);
  
  public RobotContainer() {
    if ( Robot.isSimulation() ) {
      fuelSim = new FuelSim("FuelSim");
    }
    // Setup and initialize Subsystems here
    drive_subsystem = new Drive(driverController);
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

    drive_subsystem.setDefaultCommand(
      drive_subsystem.driveWithChassisSpeedsSupplier(
        drive_subsystem.buildRelativeTurningStream()
      ));

    // SmartDashboard.putNumberArray("BlueRightClimb", new double[] {FieldConstants.BLUE_RIGHT_CLIMB_POSITION.getX(), FieldConstants.BLUE_RIGHT_CLIMB_POSITION.getY(), FieldConstants.BLUE_RIGHT_CLIMB_POSITION.getRotation().getRadians()});
    // SmartDashboard.putNumberArray("BlueLeftClimb", new double[] {FieldConstants.BLUE_LEFT_CLIMB_POSITION.getX(), FieldConstants.BLUE_LEFT_CLIMB_POSITION.getY(), FieldConstants.BLUE_LEFT_CLIMB_POSITION.getRotation().getRadians()});
    // SmartDashboard.putNumberArray("RedRightClimb", new double[] {FieldConstants.RED_RIGHT_CLIMB_POSITION.getX(), FieldConstants.RED_RIGHT_CLIMB_POSITION.getY(), FieldConstants.RED_RIGHT_CLIMB_POSITION.getRotation().getRadians()});
    // SmartDashboard.putNumberArray("RedLeftClimb", new double[] {FieldConstants.RED_LEFT_CLIMB_POSITION.getX(), FieldConstants.RED_LEFT_CLIMB_POSITION.getY(), FieldConstants.RED_LEFT_CLIMB_POSITION.getRotation().getRadians()});

    if ( Robot.isSimulation() ) {
      configureSim();
    }

    autos = new Autos(this, drive_subsystem );

    configureBindings();
    configureAuto();
  }

  /**
   * Called from Robot.disabledPeriodic so we can run disabled-mode work for
   * subsystems and autos. This centralizes the checks so heavy work in Autos
   * runs only when the chooser selection actually changes.
   */
  public void disabledPeriodic() {
    // Keep vision update behavior consistent with previous Robot.disabledPeriodic
    if (Robot.isReal()) {
      if (vision_subsystem.seesAprilTag()) {
        vision_subsystem.disabledPeriodic();
      }
    }

    if (this.autos != null) {
      // this.autos.maybeUpdateDisabledPath();
    }
  }

  private void configureAuto() {
    autos.addCommand("Shoot", this::shootGroup);
    autos.addCommand("AimShoot", this::aimShootGroup);
    autos.addCommand("ClimberUp", climber_subsystem::climbUp);
    autos.addCommand("ClimberZero", climber_subsystem::climbZero);
    autos.addCommand("ClimberDown", climber_subsystem::climb);
    autos.addCommand("Barf", this::barf);
    autos.addCommand("Intake", this::intake);
    autos.addAutos();
  }

  private void configureSim() {
    fuelSim = new FuelSim("FuelSim");
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
    BooleanSupplier noManualOverride = () -> !Dashboard.getManualOverride();

    driverController.a().whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToElevator()));
    
    // Reset Gyro
    driverController.b().whileTrue(drive_subsystem.resetOdom());

    // Lock Wheels 
    driverController.x().whileTrue(drive_subsystem.lockWheels());
    driverController.y().whileTrue(aimShootGroup());
    driverController.leftBumper().and(drive_subsystem::isOnOurSide).whileTrue(
      Commands.parallel(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToCurveStream()),
        this.shootGroup()
      ))
      .onTrue(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(true)))
      .onFalse(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(false)));

    // driverController.rightBumper().whileTrue(this.intake());

    driverController.leftTrigger()
      .whileTrue(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDefaultStream())
      );

    // Cuts the speed of the bot in half for more precise maneuvering
    // driverController.rightTrigger()
    //   .whileTrue(
    //     drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildHalfDriveStream())
    //   );
    // right trigger - shoot forwards, but only when robot is on our side of the field
    // Inform Drive when aim mode is active so indexer/intake can require being aimed before feeding
    // driverController.rightTrigger()
    //   .and(noManualOverride)
    //   .and(() -> drive_subsystem.isOnOurSide())
    //   .onTrue(Commands.runOnce(() -> drive_subsystem.setAimModeActive(true)));
    
    // driverController.rightTrigger()
    //   .and(noManualOverride)
    //   .and(() -> drive_subsystem.isOnOurSide())
    //   .onFalse(Commands.runOnce(() -> drive_subsystem.setAimModeActive(false)));
    
    // driverController.rightTrigger()
    //   .and(() -> drive_subsystem.isOnOurSide())
    //   .and(noManualOverride)
    //   .whileTrue(new ParallelCommandGroup(this.shootGroup(), drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildAimingStream())));
    
    // driverController.rightTrigger()
    //   .and(() -> !drive_subsystem.isOnOurSide())
    //   .and(noManualOverride)
    //   .whileTrue(this.shootGroup());
    
    // Commands for auto driving through trenches
    driverController.povUp().whileTrue(new DeferredCommand(drive_subsystem::driveThroughTrenchSS, Set.of(drive_subsystem)));
    driverController.povDown().whileTrue(new DeferredCommand(drive_subsystem::driveThroughTrenchOS, Set.of(drive_subsystem)));  
    driverController.povLeft()
      .and(noManualOverride)
      .whileTrue( Commands.sequence(
        new DeferredCommand( drive_subsystem::driveThroughTrenchSS, Set.of(drive_subsystem)),
        new InstantCommand(() -> drive_subsystem.setDriveToPoseActive(true)),
        new DeferredCommand(() -> drive_subsystem.driveToTargetPose(drive_subsystem.getClosestPointOnCurve(0), 0), Set.of(drive_subsystem)),
        Commands.parallel(
          drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToCurveStream()),
          this.shootGroup()
        ).withTimeout(10.0),
        new InstantCommand(() -> drive_subsystem.setDriveToPoseActive(false)),
        new DeferredCommand( drive_subsystem::driveThroughTrenchSS, Set.of(drive_subsystem))
      ));
  
      
    //Co Driver Controls
    codriverController.a().whileTrue(climber_subsystem.manualClimbUpVoltage());
    codriverController.b().whileTrue(climber_subsystem.manualClimbDownVoltage());

    // Left bumper - barf balls out of the hopper/intake
    codriverController.leftBumper().whileTrue(this.barf());

    // Right bumper - intake balls into the hopper
    codriverController.rightBumper().whileTrue(this.intake());

    //Right Trigger - manual shoot
    codriverController.rightTrigger().whileTrue(this.shootGroup());

    // Climber controls on codriver X, Y, and B
    codriverController.povUp().whileTrue(climber_subsystem.climbUp());
    
    // Y button - climb to climbing position (middle position)
    codriverController.povLeft().whileTrue(climber_subsystem.climb());

    // B button - climb down\][
    ]
    codriverController.povDown().onTrue(climber_subsystem.climbZero());
  }

  public Command getAutonomousCommand() {
    return this.autos.getAutonomousCommand();
  }

  public Vision getVision() {
    return this.vision_subsystem;
  }

  private Command shootGroup(){
    BooleanSupplier manualOverride = () -> Dashboard.getManualOverride();

    BooleanSupplier angleDriveApprovesOfShooting = () ->
      drive_subsystem.isAimModeActive() && (Dashboard.getManualOverride() || drive_subsystem.isAimedAtHub(Math.toRadians(6)));

    BooleanSupplier curveDriveApprovesOfShooting = () -> {
        if (manualOverride.getAsBoolean()) {
          return true; // If manual override is active, don't gate shooting at all
        }
        if (!drive_subsystem.isDriveToPoseActive()) {
          return true;
        }
        // When autodrive is active, allow shooting only if the robot's distance
        // to the hub is within a tolerance of the midRange used to generate the curve.
        double midRange = RobotConstants.ShooterSubsystemConstants.midRange;
        double posTol = midRange * 0.05; // 5% tolerance around midRange
        double hubDist = drive_subsystem.getPose().getTranslation().getDistance(
            RobotConstants.FieldConstants.hubPosition.get());
        boolean distOk = Math.abs(hubDist - midRange) <= posTol;
        boolean angleOk = drive_subsystem.isAimedAtHub(Math.toRadians(6.0));
        return distOk && angleOk;
    };

    // TODO check why curve isn't approving of shooting
    BooleanSupplier doShootFeeding = () -> {
      return shooter_subsystem.isUpToSpeed() 
        && (
          (
            // We don't check for correct pose or angle when not using either mode
            ((angleDriveApprovesOfShooting.getAsBoolean() || !drive_subsystem.isAimModeActive()) 
            && (curveDriveApprovesOfShooting.getAsBoolean() || !drive_subsystem.isDriveToPoseActive())
          )
          // Manual override stops from requiring pose or angle while in those modes
          || manualOverride.getAsBoolean()
        )
      );
    };
    
    return Commands.parallel(
      shooter_subsystem.Shoot(),
      shaker_subsystem.Shake(),
      Commands.sequence(
        Commands.parallel(
          new WaitUntilCommand(shooter_subsystem::isUpToSpeed)
        ).withTimeout(3),
        Commands.parallel(
          indexer_subsystem.IndexerOut(doShootFeeding),
          intake_subsystem.IntakeIn()
        )
      )
    );
  }

  private Command aimShootGroup(){
    // TODO check why curve isn't approving of shooting
    BooleanSupplier doShootFeeding = () -> {
      return shooter_subsystem.isUpToSpeed();
    };
    
    return Commands.parallel(
      shooter_subsystem.Shoot(),
      shaker_subsystem.Shake(),
      drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildAimingStream()),
      Commands.sequence(
        Commands.parallel(
          new WaitUntilCommand(shooter_subsystem::isUpToSpeed)
        ).withTimeout(3),
        Commands.parallel(
          indexer_subsystem.IndexerOut(doShootFeeding),
          intake_subsystem.IntakeIn()
        )
      )
    );
  }
  private Command barf(){
    return Commands.parallel(
      intake_subsystem.IntakeBarf(),
      shaker_subsystem.Shake(),
      shooter_subsystem.ShootBackwards().withTimeout(1),
      Commands.sequence(
        indexer_subsystem.IndexerIn().withTimeout(0.25),
        indexer_subsystem.IndexerOut()
      )
    );
  }

  private Command intake(){
    return Commands.parallel(
      intake_subsystem.IntakeIn(),
      indexer_subsystem.IndexerIn()
    );
  }
}
