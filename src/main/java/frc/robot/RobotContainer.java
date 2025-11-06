// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Drive;
import frc.robot.Autos;
import frc.robot.Constants.ControllerConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Vision;

public class RobotContainer {

  private final Autos autos;
  private final Drive driveSubsystem;
  private final Vision vision;
  private final CommandXboxController driverController;
  private final CommandXboxController codriverController;

  public RobotContainer() {
    // Setup and initialize Subsystems here
    this.driveSubsystem = new Drive();
    this.vision = this.driveSubsystem.getVision();
    this.autos = new Autos(this, this.driveSubsystem );

    // Configure all remote bindings
    this.driverController = new CommandXboxController(0);
    this.codriverController = new CommandXboxController(1);
    this.configureBindings();
    this.configureDriving();
    SmartDashboard.putBoolean("Sim Running", false );
  }

  private void configureBindings() {
  }

  public void configureDriving() {
    DoubleSupplier getTranslationX = () -> MathUtil.applyDeadband(this.driverController.getLeftY(),
        ControllerConstants.deadbandX);
    DoubleSupplier getTranslationY = () -> MathUtil.applyDeadband(this.driverController.getLeftX(),
        ControllerConstants.deadbandY);
    DoubleSupplier getHeadingX = this.driverController::getRightX;
    DoubleSupplier getHeadingY = this.driverController::getRightY;

    Command defaultDrive = driveSubsystem.driveCommand(
        getTranslationX,
        getTranslationY,
        getHeadingX,
        getHeadingY
    );

    this.driveSubsystem.setDefaultCommand(defaultDrive);
  }

  public Command getAutonomousCommand() {
    return this.autos.getAutonomousCommand();
  }
}
