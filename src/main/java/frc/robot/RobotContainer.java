// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.ControllerConstants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shaker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Vision.RealVision;
import frc.robot.Subsystems.Vision.SimVision;
import frc.robot.Subsystems.Vision.Vision;

public class RobotContainer {

  private final Autos autos;
  private final Drive drive_subsystem;


  private final Vision vision_subsystem;
  private final Intake intake_subsystem;
  private final Indexer indexer_subsystem;
  private final Shooter shooter_subsystem;
  private final Climber climber_subsystem;
  // private final Shaker shaker_subsystem;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController codriverController = new CommandXboxController(1);

  public RobotContainer() {
    // Setup and initialize Subsystems here
    drive_subsystem = new Drive();
    vision_subsystem = drive_subsystem.getVision();
    shooter_subsystem = new Shooter();
    intake_subsystem = new Intake(shooter_subsystem);
    indexer_subsystem = new Indexer( shooter_subsystem, intake_subsystem );
    climber_subsystem = new Climber();
    // shaker_subsystem = new Shaker();

    autos = new Autos(this, drive_subsystem );

    configureBindings();
    configureDriving();
  }

  private void configureBindings() {
    // Pathfinding command to drive to a specific pose on button press
    // driverController.a().whileTrue( 
    //   edu.wpi.first.wpilibj2.command.Commands.deferredProxy(
    //     () -> drive_subsystem.dummyDrivePose()
    //   )
    // ); 

    // SysId complete routine for shooter characterization - runs all 4 tests in sequence
    driverController.a().onTrue(shooter_subsystem.getCompleteSysIdRoutine());

    // Reset odometry to current Limelight pose
    // driverController.b().onTrue(drive_subsystem.resetOdometryWithVision());

    // Shooter bindings on codriver controller
    // Left trigger - shoot forwards
    // driverController.leftTrigger().whileTrue( new ParallelCommandGroup(
    //     shooter_subsystem.Shoot(),
    //     shaker_subsystem.shake()
    // ));
    
    // // Right trigger - shoot backwards
    // driverController.rightTrigger().whileTrue( new ParallelCommandGroup( shooter_subsystem.ShootBackwards(), shaker_subsystem.shake()));
    
    // // Intake controls on codriver bumpers
    // driverController.leftBumper().whileTrue(intake_subsystem.IntakeOn());
    
    // driverController.rightBumper().whileTrue( new ParallelCommandGroup( intake_subsystem.IntakeBarf(), shaker_subsystem.shake()));
    driverController.leftTrigger().whileTrue( new ParallelCommandGroup(
        shooter_subsystem.Shoot()
    ));
    
    // Right trigger - shoot backwards
    driverController.rightTrigger().whileTrue( new ParallelCommandGroup( shooter_subsystem.ShootBackwards()));
    
    // Intake controls on codriver bumpers
    driverController.leftBumper().whileTrue(intake_subsystem.IntakeOn());
    
    driverController.rightBumper().whileTrue( new ParallelCommandGroup( intake_subsystem.IntakeBarf()));


    // Climber controls on codriver X, Y, and B
    codriverController.x().whileTrue(climber_subsystem.climbUp());
    
    // Y button - climb to climbing position (middle position)
    codriverController.y().whileTrue(climber_subsystem.climb());
    
    // B button - climb down
    codriverController.b().whileTrue(climber_subsystem.climbZero());

    // Manual voltage control for climber on codriver triggers
    codriverController.leftTrigger().whileTrue(climber_subsystem.manualClimbUpVoltage());
    codriverController.rightTrigger().whileTrue(climber_subsystem.manualClimbDownVoltage());
  }

  public void configureDriving() {
    DoubleSupplier getTranslationX = () -> MathUtil.applyDeadband(this.driverController.getLeftY(),
        ControllerConstants.deadbandX);
    DoubleSupplier getTranslationY = () -> MathUtil.applyDeadband(this.driverController.getLeftX(),
        ControllerConstants.deadbandY);
    DoubleSupplier getHeadingX = () -> -1 * driverController.getRightX();
    DoubleSupplier getHeadingY = () -> -1 * driverController.getRightY();

    Command defaultDrive = drive_subsystem.driveCommand(
        getTranslationX,
        getTranslationY,
        getHeadingX,
        getHeadingY
    );

    drive_subsystem.setDefaultCommand(defaultDrive);
  }

  public Command getAutonomousCommand() {
    return this.autos.getAutonomousCommand();
  }
}
