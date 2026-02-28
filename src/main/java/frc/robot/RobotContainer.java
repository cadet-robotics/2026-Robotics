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
  
    drive_subsystem.setDefaultCommand(
      drive_subsystem.driveWithChassisSpeedsSupplier(
        drive_subsystem.buildDefaultStream()
      ));

    // Shoots, Shakes, and Aims
    driverController.rightTrigger()
      .whileTrue( 
        new ParallelCommandGroup(
          shooter_subsystem.Shoot(),
          drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildAimingStream())
        ));
    
    // Uses Relative Turning
    driverController.leftTrigger()
      .whileTrue(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildRelativeTurningStream())
      );

    // Drives to a point on the curve (the arc where we are best equipped to shoot accurately)
    driverController.a()
      .whileTrue(
        drive_subsystem.driveWithChassisSpeedsSupplier(drive_subsystem.buildDriveToCurveStream())
      );

    // Barf
    driverController.rightBumper()
      .whileTrue(intake_subsystem.IntakeBarf());

    // Intake controls on codriver bumpers
    driverController.leftBumper().whileTrue(intake_subsystem.IntakeOn());

    driverController.y().whileTrue(drive_subsystem.resetOdom());

    driverController.x().whileTrue(drive_subsystem.driveToClimb());

    driverController.b().whileTrue(drive_subsystem.resetOdom());

    codriverController.povLeft().whileTrue(climber_subsystem.manualClimbUpVoltage());
    codriverController.povRight().whileTrue(climber_subsystem.manualClimbDownVoltage());
  }

  public Command getAutonomousCommand() {
    return this.autos.getAutonomousCommand();
  }

}
