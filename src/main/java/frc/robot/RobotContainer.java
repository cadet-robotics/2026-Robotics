// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private Drive driveSubsystem;
  private CommandXboxController driverController;
  private CommandXboxController codriverController;

  public RobotContainer() {
    // Setup and Initalize Subsystems here
    driveSubsystem = new Drive();

    configureBindings();
  }

  private void configureBindings() {
    this.driverController = new CommandXboxController(0);
    this.codriverController = new CommandXboxController(1);
  }

  public void configureDriving() {
    DoubleSupplier getTranlationX = () -> MathUtil.applyDeadband(this.driverController.getLeftY(),
        ControllerConstants.deadbandX);
    DoubleSupplier getTranlationY = () -> MathUtil.applyDeadband(this.driverController.getLeftX(),
        ControllerConstants.deadbandY);
    DoubleSupplier getHeadingX = () -> this.driverController.getRightX();
    DoubleSupplier getHeadingY = () -> this.driverController.getRightY();

    Command defaultDrive = driveSubsystem.driveCommand(
        getTranlationX,
        getTranlationY,
        getHeadingX,
        getHeadingY);

    this.driveSubsystem.setDefaultCommand(defaultDrive);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
