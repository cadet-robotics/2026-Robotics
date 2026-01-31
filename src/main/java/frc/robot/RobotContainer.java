// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Subsystems.Drive;

public class RobotContainer {

  private final Autos autos;
  private final Drive drive_subsystem;


  // private final Vision vision_subsystem;
  // private final Intake intake_subsystem;
  // private final Indexer indexer_subsystem;
  // private final Shooter shooter_subsystem;

  private final CommandXboxController driverController;
  private final CommandXboxController codriverController;

  public RobotContainer() {
    // Setup and initialize Subsystems here
    drive_subsystem = new Drive();
    // vision_subsystem = drive_subsystem.getVision();
    // shooter_subsystem = new Shooter();
    // intake_subsystem = new Intake();
    // indexer_subsystem = new Indexer( shooter_subsystem, intake_subsystem );

    autos = new Autos(this, drive_subsystem );
    
    // Configure all remote bindings
    driverController = new CommandXboxController(0);
    codriverController = new CommandXboxController(1);
    configureBindings();
    configureDriving();
  }

  private void configureBindings() {
    //new Trigger( () -> driverController.getLeftX() != 0 ).whileTrue( drive_subsystem.dummyDrivePose() );
    driverController.a().whileTrue( drive_subsystem.dummyDrivePose() );
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
    return drive_subsystem.getDefaultCommand();
  }
}
