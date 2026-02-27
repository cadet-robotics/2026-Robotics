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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ControllerConstants;
import frc.robot.Libs.FuelSim;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
// import frc.robot.Subsystems.Shaker;
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
  // private final Shaker shaker_subsystem;

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
    // shaker_subsystem = new Shaker();

    autos = new Autos(this, drive_subsystem );

  // Default lead target velocity (m/s) for testing — editable on SmartDashboard
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Lead/TargetVx", 0.0);
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Lead/TargetVy", 0.0);
  // Tunable multiplier to increase/decrease computed lead time (default >1 to increase lead)
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Lead/TimeMultiplier", 1.25);

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
    
    // driverController.leftBumper().whileTrue(intake_subsystem.IntakeOn());
    
    // driverController.rightBumper().whileTrue( new ParallelCommandGroup( intake_subsystem.IntakeBarf(), shaker_subsystem.shake()));
    driverController.rightTrigger().whileTrue( new ParallelCommandGroup(
        shooter_subsystem.Shoot()
    ));
    
    // Right trigger - shoot backwards
    driverController.leftTrigger().whileTrue( new ParallelCommandGroup( shooter_subsystem.ShootBackwards()));
    
    // Intake controls on codriver bumpers
    driverController.leftBumper().whileTrue(intake_subsystem.IntakeOn());
    
    driverController.rightBumper().whileTrue( new ParallelCommandGroup( intake_subsystem.IntakeBarf()));

    driverController.a().whileTrue(drive_subsystem.resetOdom());

    codriverController.b().whileTrue(drive_subsystem.driveToClimb());

    codriverController.povLeft().whileTrue(climber_subsystem.manualClimbUpVoltage());
    codriverController.povRight().whileTrue(climber_subsystem.manualClimbDownVoltage());
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
      // Compute the heading angle from robot->hub, add PI to point opposite, and provide
      // cos/sin as the heading axis. Using angle avoids sign ambiguities in axis ordering.
      .withControllerHeadingAxis(
        () -> {
          // Lead calculation moved here: compute where the robot should face to intercept
          edu.wpi.first.math.geometry.Pose2d robotPose = drive_subsystem.getPose();
          var target = RobotConstants.FieldConstants.hub;
          // test target velocity (m/s) from SmartDashboard (ground-relative)
          double tvx = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TargetVx", 0.0);
          double tvy = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TargetVy", 0.0);

          // Simple time-distance model: at 2 meters projectile takes 1.5s -> t = k * distance
          double k = 2 / 2.0; // seconds per meter

          double relX = target.getX() - robotPose.getX();
          double relY = target.getY() - robotPose.getY();

          // Solve for t using relation t = k * ||rel + v_t * t||.
          // Rearranged and squared it becomes: (v_t·v_t - 1/k^2) t^2 + 2 (rel·v_t) t + rel·rel = 0
          double vx = tvx;
          double vy = tvy;
          double v2 = vx * vx + vy * vy;
          double invk = 1.0 / k;
          double invk2 = invk * invk;
          double a = v2 - invk2;
          double b = 2.0 * (relX * vx + relY * vy);
          double c = relX * relX + relY * relY;
          double t = Double.NaN;
          if (Math.abs(a) < 1e-12) {
            // Linear case: b t + c = 0
            if (Math.abs(b) > 1e-12) {
              double tr = -c / b;
              if (tr > 1e-6) t = tr;
            }
          } else {
            double disc = b * b - 4.0 * a * c;
            if (disc >= 0.0) {
              double sq = Math.sqrt(disc);
              double t1 = (-b - sq) / (2.0 * a);
              double t2 = (-b + sq) / (2.0 * a);
              double best = Double.POSITIVE_INFINITY;
              if (t1 > 1e-6 && t1 < best) best = t1;
              if (t2 > 1e-6 && t2 < best) best = t2;
              if (best < Double.POSITIVE_INFINITY) t = best;
            }
          }
          if (Double.isNaN(t) || t <= 1e-6) {
            // fallback: assume stationary target and use proportional time
            double dist = Math.hypot(relX, relY);
            t = k * dist;
          }
          // Apply a tunable time multiplier to increase/decrease lead
          double timeMul = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TimeMultiplier", 1.0);
          t *= timeMul;
          if (Double.isNaN(t) || t <= 1e-6) {
            double dist = Math.hypot(relX, relY);
            t = k * dist;
          }
          // Publish computed lead time for debugging
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Lead/ComputedTime", t);

          double ix = target.getX() + tvx * t;
          double iy = target.getY() + tvy * t;

          double angle = Math.atan2(iy - robotPose.getY(), ix - robotPose.getX()); // face intercept
          return Math.sin(angle+Math.PI);
        },
        () -> {
          edu.wpi.first.math.geometry.Pose2d robotPose = drive_subsystem.getPose();
          var target = RobotConstants.FieldConstants.hub;
          double tvx = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TargetVx", 0.0);
          double tvy = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TargetVy", 0.0);
          // Simple time-distance model: at 2 meters projectile takes 1.5s -> t = k * distance
          double k = 2 / 2.0; // seconds per meter
          double relX = target.getX() - robotPose.getX();
          double relY = target.getY() - robotPose.getY();
          // Solve for t using relation t = k * ||rel + v_t * t||.
          double vx = tvx;
          double vy = tvy;
          double v2 = vx * vx + vy * vy;
          double invk = 1.0 / k;
          double invk2 = invk * invk;
          double a = v2 - invk2;
          double b = 2.0 * (relX * vx + relY * vy);
          double c = relX * relX + relY * relY;
          double t = Double.NaN;
          if (Math.abs(a) < 1e-12) {
            if (Math.abs(b) > 1e-12) {
              double tr = -c / b;
              if (tr > 1e-6) t = tr;
            }
          } else {
            double disc = b * b - 4.0 * a * c;
            if (disc >= 0.0) {
              double sq = Math.sqrt(disc);
              double t1 = (-b - sq) / (2.0 * a);
              double t2 = (-b + sq) / (2.0 * a);
              double best = Double.POSITIVE_INFINITY;
              if (t1 > 1e-6 && t1 < best) best = t1;
              if (t2 > 1e-6 && t2 < best) best = t2;
              if (best < Double.POSITIVE_INFINITY) t = best;
            }
          }
          if (Double.isNaN(t) || t <= 1e-6) {
            double dist = Math.hypot(relX, relY);
            t = k * dist;
          }
          // Apply a tunable time multiplier to increase/decrease lead
          double timeMul2 = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Lead/TimeMultiplier", 1.0);
          t *= timeMul2;
          if (Double.isNaN(t) || t <= 1e-6) {
            double dist = Math.hypot(relX, relY);
            t = k * dist;
          }
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Lead/ComputedTime2", t);
          double ix = target.getX() + tvx * t;
          double iy = target.getY() + tvy * t;
          double angle = Math.atan2(iy - robotPose.getY(), ix - robotPose.getX()); // face intercept
          return Math.cos(angle+Math.PI);
        }
      )
      .headingWhile(true);

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
      drive_subsystem.driveWithChassisSpeedsSupplier(defaultStream)
    );

    BooleanSupplier relativeDriveCondition = driverController.leftTrigger()::getAsBoolean;
    BooleanSupplier aimDriveCondition = driverController.rightTrigger()::getAsBoolean;
    BooleanSupplier autoDriveCondition = driverController.a()::getAsBoolean;

  // We do a little bit of binding here which might be bad idk
    driverController
      .leftTrigger()
      .and(() -> {return !aimDriveCondition.getAsBoolean() && !autoDriveCondition.getAsBoolean();})
      .whileTrue(drive_subsystem.driveWithChassisSpeedsSupplier(relativeTurning));

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
    // Keep Drive informed when the drive-to-pose input stream is active so other subsystems
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
