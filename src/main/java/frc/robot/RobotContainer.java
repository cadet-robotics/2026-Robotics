// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ControllerConstants;
import frc.robot.Constants.RobotConstants.FieldConstants;
import frc.robot.Libs.FuelSim;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shaker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Vision.Vision;
import swervelib.SwerveInputStream;

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
    drive_subsystem = new Drive();
    vision_subsystem = drive_subsystem.getVision();
    shooter_subsystem = new Shooter(fuelSim);
      // Register robot pose supplier for simulation projectile manager
    intake_subsystem = new Intake(shooter_subsystem, drive_subsystem);
    indexer_subsystem = new Indexer( shooter_subsystem, intake_subsystem, drive_subsystem );
    climber_subsystem = new Climber();
    shaker_subsystem = new Shaker();

    autos = new Autos(this, drive_subsystem );

  // Default lead target velocity (m/s) for testing — editable on SmartDashboard
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Lead/TargetVx", 0.0);
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Lead/TargetVy", 0.0);
  // Tunable multiplier to increase/decrease computed lead time (default >1 to increase lead)
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Lead/TimeMultiplier", 1.25);
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("ActualSubsystem",drive_subsystem);

    configureBindings();
    configureDriving();
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
    // Pathfinding command to drive to a specific pose on button press
    // driverController.a().whileTrue( 
    //   edu.wpi.first.wpilibj2.command.Commands.deferredProxy(
    //     () -> drive_subsystem.dummyDrivePose()
    //   )
    // ); 

    // SysId complete routine for shooter characterization - runs all 4 tests in sequence
    // driverController.a().onTrue(shooter_subsystem.getCompleteSysIdRoutine());

    // Reset odometry to current Limelight pose
    // driverController.b().onTrue(drive_subsystem.resetOdometryWithVision());

    // Shooter bindings on codriver controller
    // right trigger - shoot forwards
    driverController.rightTrigger().whileTrue( new ParallelCommandGroup(
        shooter_subsystem.Shoot(),
        shaker_subsystem.shake()
    ));
    
    // // Right trigger - shoot backwards
    // driverController.rightTrigger().whileTrue( new ParallelCommandGroup( shooter_subsystem.ShootBackwards(), shaker_subsystem.shake()));
    
    // driverController.leftBumper().whileTrue(intake_subsystem.IntakeOn());
    
    driverController.rightBumper().whileTrue( new ParallelCommandGroup( intake_subsystem.IntakeBarf(), shaker_subsystem.shake()));
    // driverController.rightTrigger().whileTrue( new ParallelCommandGroup(
    //     shooter_subsystem.Shoot()
    // ));
    
    // // Right trigger - shoot backwards
    // driverController.leftTrigger().whileTrue( new ParallelCommandGroup( shooter_subsystem.ShootBackwards()));
    
    // Intake controls on codriver bumpers
    driverController.leftBumper().whileTrue(intake_subsystem.IntakeOn());
    
    // driverController.rightBumper().whileTrue( new ParallelCommandGroup( intake_subsystem.IntakeBarf()));

    driverController.y().whileTrue(drive_subsystem.resetOdom());

    driverController.x().whileTrue(drive_subsystem.driveToClimb());

    driverController.b().whileTrue(drive_subsystem.resetOdom());
    // Climber controls on codriver X, Y, and B
    // codriverController.x().whileTrue(climber_subsystem.climbUp());
    
    // Y button - climb to climbing position (middle position)
    // codriverController.y().whileTrue(climber_subsystem.climb());
    
    // B button - climb down
    // codriverController.b().whileTrue(climber_subsystem.climbZero());

    // Manual voltage control for climber on codriver triggers
    // codriverController.leftTrigger().whileTrue(climber_subsystem.manualClimbUpVoltage());
    // driverController.y().whileTrue(climber_subsystem.manualClimbDownVoltage());
  }

  public void configureDriving() {
    DoubleSupplier getTranslationX = () -> -1 * MathUtil.applyDeadband(this.driverController.getLeftY(),
        ControllerConstants.deadbandX);
    DoubleSupplier getTranslationY = () -> -1 * MathUtil.applyDeadband(this.driverController.getLeftX(),
        ControllerConstants.deadbandY);
    DoubleSupplier getHeadingX = () -> -1 * driverController.getRightX();
    DoubleSupplier getHeadingY = () -> -1 * driverController.getRightY();

    SwerveInputStream baseStream = SwerveInputStream.of(drive_subsystem.getSwerveDrive(), getTranslationX, getTranslationY)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true)
      .deadband(0.12);

    // The default SIS
    SwerveInputStream defaultStream = baseStream.copy()
      .withControllerHeadingAxis(getHeadingX, getHeadingY)
      .headingWhile(true);

    // A SIS with relative turning
    SwerveInputStream relativeTurning = baseStream
      .copy()
      .withControllerRotationAxis(() -> -1 * driverController.getRightX());

    // A SIS which looks at the hub but uses a heading vector that's opposite the hub direction
    SwerveInputStream looking = baseStream
      .copy()
      .aim(FieldConstants.hub.rotateAround(drive_subsystem.getPose().getTranslation(), Rotation2d.k180deg))
      .aimWhile(true);

    // A SIS which uses the auto drive to pose to align to a point on the curve
    SwerveInputStream autoDrive = baseStream
      .copy()
      .driveToPose(
        () -> drive_subsystem.getClosestPointOnCurve(driverController.getLeftX()),
        RobotConstants.DriveSubsystemConstants.translationProfiledController, 
        RobotConstants.DriveSubsystemConstants.rotationProfiledController
      )
      .driveToPoseEnabled(true);

    // Will use default by default
    drive_subsystem.setDefaultCommand(
      drive_subsystem.driveWithChassisSpeedsSupplier(defaultStream).withName("headingDrive")
    );

    BooleanSupplier relativeDriveCondition = driverController.leftTrigger()::getAsBoolean;
    BooleanSupplier aimDriveCondition = driverController.rightTrigger()::getAsBoolean;
    BooleanSupplier autoDriveCondition = driverController.a()::getAsBoolean;

  // We do a little bit of binding here which might be bad idk
    driverController
      .leftTrigger()
      .and(driverController.a().negate())
      .and(() -> !autoDriveCondition.getAsBoolean())
      .whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(relativeTurning).withName("rotationDrive"));

    driverController
      .rightTrigger()
      .and(() -> {return !autoDriveCondition.getAsBoolean();})
      .whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(looking));
    // Inform Drive when aim mode is active so indexer/intake can require being aimed before feeding
    driverController.rightTrigger().onTrue(Commands.runOnce(() -> drive_subsystem.setAimModeActive(true)));
    driverController.rightTrigger().onFalse(Commands.runOnce(() -> drive_subsystem.setAimModeActive(false)));
    
    driverController
      .a()
      .whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(autoDrive));
    // // Keep Drive informed when the drive-to-pose input stream is active so other subsystems
    // can gate behavior (indexer/intake) based on whether we're at the pose.
    driverController.a().onTrue(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(true)));
    driverController.a().onFalse(Commands.runOnce(() -> drive_subsystem.setDriveToPoseActive(false)));

    // Button to fire a lead shot using SmartDashboard-specified target velocity (for testing)
    driverController.x().onTrue(
      Commands.runOnce(() -> {
        double vx = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TargetVx", 0.0);
        double vy = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TargetVy", 0.0);
        // Subtract robot field velocity so target velocity is relative to the ground minus robot motion.
        var speeds = drive_subsystem.getSwerveDrive().getFieldVelocity();
        double rvx = speeds.vxMetersPerSecond;
        double rvy = speeds.vyMetersPerSecond;
        shooter_subsystem.leadAndLaunch(drive_subsystem.getPose(), RobotConstants.FieldConstants.hub, new edu.wpi.first.math.geometry.Translation2d(vx - rvx, vy - rvy));
      }, shooter_subsystem)
    );
  }

  public Command getAutonomousCommand() {
    return this.autos.getAutonomousCommand();
  }

}
