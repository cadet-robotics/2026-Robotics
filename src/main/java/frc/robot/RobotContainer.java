// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.ControllerConstants;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Vision.Vision;

public class RobotContainer {

  private final Autos autos;
  private final Drive drive_subsystem;


  private final Vision vision_subsystem;
  private final Intake intake_subsystem;
  private final Indexer indexer_subsystem;
  private final Shooter shooter_subsystem;

  private final CommandXboxController driverController;
  private final CommandXboxController codriverController;

  public RobotContainer() {
    // Setup and initialize Subsystems here
    drive_subsystem = new Drive();
    vision_subsystem = drive_subsystem.getVision();
    shooter_subsystem = new Shooter();
    intake_subsystem = new Intake();
    indexer_subsystem = new Indexer( shooter_subsystem, intake_subsystem );

    autos = new Autos(this, drive_subsystem );
    
    // Configure all remote bindings
    driverController = new CommandXboxController(0);
    codriverController = new CommandXboxController(1);
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

    // Shooter bindings on codriver controller
    // Left trigger - shoot forwards
    // codriverController.leftTrigger().whileTrue(shooter_subsystem.Shoot())
    //                                 .onFalse(shooter_subsystem.StopShooting());
    
    // // Right trigger - shoot backwards
    // codriverController.rightTrigger().whileTrue(shooter_subsystem.ShootBackwards())
    //                                  .onFalse(shooter_subsystem.StopShooting());
    
    // Y button - stop shooter
    // codriverController.y().onTrue(shooter_subsystem.StopShooting());
    
    // Left D-pad - manual spin forward at low speed
    // driverController.leftBumper().whileTrue(shooter_subsystem.ManualSpinForward());
    
    // // Right D-pad - manual spin backward at low speed
    // driverController.rightBumper().whileTrue(shooter_subsystem.ManualSpinBackward());

    // // Indexer bindings on codriver controller
    // // Left bumper - manual indexer forward (slow)
    // driverController.leftTrigger().whileTrue(indexer_subsystem.manualForward());
    
    // // Right bumper - manual indexer backward (slow)
    // driverController.rightTrigger().whileTrue(indexer_subsystem.manualBackward());

    // driverController.x().whileTrue(this.intake_subsystem.SetIntakeOn());
    // driverController.b().whileTrue(this.intake_subsystem.SetIntakeOff());

    
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

    driverController.a().whileTrue(this.drive_subsystem.faceHub(getTranslationX, getTranslationY));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = autos.getAutonomousCommand();
    // If no auto is selected, return the default command
    return autoCommand != null ? autoCommand : drive_subsystem.getDefaultCommand();
  }
}
