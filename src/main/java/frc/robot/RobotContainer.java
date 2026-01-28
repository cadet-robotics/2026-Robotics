// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.*;
import frc.robot.Constants.ControllerConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    this.drive_subsystem = new Drive();
    this.vision_subsystem = this.drive_subsystem.getVision();
    this.shooter_subsystem = new Shooter();
    this.intake_subsystem = new Intake();
    this.indexer_subsystem = new Indexer( this.shooter_subsystem, this.intake_subsystem );

    this.autos = new Autos(this, this.drive_subsystem );
    
    // Configure all remote bindings
    this.driverController = new CommandXboxController(0);
    this.codriverController = new CommandXboxController(1);
    this.configureBindings();
    this.configureDriving();
  }

  private void configureBindings() {
    //new Trigger( () -> this.driverController.getLeftX() != 0 ).whileTrue( this.drive_subsystem.dummyDrivePose() );
    this.driverController.a().whileTrue( this.drive_subsystem.dummyDrivePose() );
  }

  public void configureDriving() {
    DoubleSupplier getTranslationX = () -> MathUtil.applyDeadband(this.driverController.getLeftY(),
        ControllerConstants.deadbandX);
    DoubleSupplier getTranslationY = () -> MathUtil.applyDeadband(this.driverController.getLeftX(),
        ControllerConstants.deadbandY);
    DoubleSupplier getHeadingX = () -> -1 * this.driverController.getRightX();
    DoubleSupplier getHeadingY = () -> -1 * this.driverController.getRightY();

    Command defaultDrive = drive_subsystem.driveCommand(
        getTranslationX,
        getTranslationY,
        getHeadingX,
        getHeadingY
    );

    this.drive_subsystem.setDefaultCommand(defaultDrive);
  }

  public Command getAutonomousCommand() {
    return this.drive_subsystem.getDefaultCommand();
  }
}
