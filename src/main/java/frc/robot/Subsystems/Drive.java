package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Configuration.DriveSubsystemConfiguration;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import frc.robot.Robot;
import frc.robot.Subsystems.Vision.RealVision;
import frc.robot.Subsystems.Vision.SimVision;
import frc.robot.Subsystems.Vision.Vision;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import static edu.wpi.first.units.Units.*;

public class Drive extends CSubsystem {
    // TODO: Configure robot details
    private static final double maxSpeed = Units.feetToMeters(6);
    private SwerveDrive swerveDrive;
    private Vision vision;

    public Drive() {

        if (Robot.isReal()) {
            this.vision = new RealVision(this);
        } else {
            this.vision = new SimVision(this);
        }

        // Temp starting positions for sim
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                Meter.of(4)),
                Rotation2d.fromDegrees(180))
                : new Pose2d(new Translation2d(Meter.of(16),
                        Meter.of(4)),
                        Rotation2d.fromDegrees(0));

        this.configureSwerveObjects(startingPose);
        DriveSubsystemConfiguration.configurePathPlanner(this, this.swerveDrive);
    }

    public void configureSwerveObjects(Pose2d startingPose) {
        // Initialize the swerve drive object with the configs in the deploy directory
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            this.swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Drive.maxSpeed, startingPose);
        } catch (IOException e) {
            System.out.println("Failed to load necessary files for swerve drive.");
            throw new RuntimeException(e);
        }

        this.swerveDrive.resetOdometry(startingPose);
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }

    public Vision getVision() {
        return this.vision;
    }

    public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }

    /**
     * Command to drive the robot using translate values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        return run(() -> {

            Translation2d scaledInputs = SwerveMath
                    .scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 3);
            // Make the robot move
            this.swerveDrive.driveFieldOriented(
                    swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                            headingX.getAsDouble(),
                            headingY.getAsDouble(),
                            swerveDrive.getOdometryHeading().getRadians(),
                            swerveDrive.getMaximumChassisVelocity()));
        });
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose2d) {
        swerveDrive.resetOdometry(pose2d);
    }

    /**
     * Takes in a pose from the limelight and uses it to update the swerve modules
     * position.
     *
     * @param pose Current pose of the robot
     */
    public void updatePose(Pose2d pose) {
        swerveDrive.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }

    /**
     * Command to drive the robot using translate values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public CCommand driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return this.cCommand("DriveSubsysem.DefaultDrive").onExecute(() -> {
            double vX = translationX.getAsDouble();
            double vY = translationY.getAsDouble();
            double omega = angularRotationX.getAsDouble();
            
            // Debug output
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/InputX", vX);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/InputY", vY);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/InputOmega", omega);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Drive/CommandRunning", true);
            
            // Make the robot move
            swerveDrive.drive(
                new Translation2d(
                    vX * swerveDrive.getMaximumChassisVelocity(),
                    vY * swerveDrive.getMaximumChassisVelocity()),
                omega * swerveDrive.getMaximumChassisAngularVelocity(),
                true,
                false);
        });
    }

    /**
     * Command to drive the robot using translate values and heading as angular
     * velocity at half speed.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public CCommand halfDriveCommand(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return this.driveCommand(translationX, translationY, angularRotationX)
                .onInitialize(() -> {
                    this.swerveDrive.setMaximumAllowableSpeeds(
                            this.swerveDrive.getMaximumChassisVelocity() / 2,
                            this.swerveDrive.getMaximumChassisAngularVelocity());
                }).onEnd(() -> {
                    this.swerveDrive.setMaximumAllowableSpeeds(
                            this.swerveDrive.getMaximumChassisVelocity() / 2,
                            this.swerveDrive.getMaximumChassisAngularVelocity());
                });
    }
    
    /**
     * Command to drive to a target pose using PathPlanner's pathfinding.
     * 
     * @param targetPose The target pose to drive to
     * @param endVelocity The desired velocity at the end of the path (m/s)
     * @return Command that pathfinds to the target pose
     */
    public Command driveToTargetPose(Pose2d targetPose, double endVelocity) {
        PathConstraints constraints = new PathConstraints(
            this.swerveDrive.getMaximumChassisVelocity(),      // Max velocity (m/s)
            4.0,                                                // Max acceleration (m/s^2)
            this.swerveDrive.getMaximumChassisAngularVelocity(), // Max angular velocity (rad/s)
            Math.PI * 4.0                                       // Max angular acceleration (rad/s^2)
        );
        
        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            endVelocity  // Goal velocity at end
        );
    }

    /**
     * Command to drive to a target pose using PathPlanner's pathfinding.
     * Supplier version for dynamic targets.
     * 
     * @param pose2dSupplier Supplier that provides the target pose
     * @param endVelocity The desired velocity at the end of the path (m/s)
     * @return Command that pathfinds to the target pose
     */
    public Command driveToTargetPose(Supplier<Pose2d> pose2dSupplier, double endVelocity) {
        PathConstraints constraints = new PathConstraints(
            this.swerveDrive.getMaximumChassisVelocity(),
            4.0,
            this.swerveDrive.getMaximumChassisAngularVelocity(),
            Math.PI * 4.0
        );
        
        return AutoBuilder.pathfindToPose(
            pose2dSupplier.get(),
            constraints,
            endVelocity
        );
    }

    /**
     * Command to drive to a target pose and stop.
     * 
     * @param pose2dSupplier Supplier that provides the target pose
     * @return Command that pathfinds to the target pose and stops
     */
    public Command driveToTargetPose(Supplier<Pose2d> pose2dSupplier) {
        return driveToTargetPose(pose2dSupplier, 0.0);
    }

    public Command dummyDrivePose() {
        return this.driveToTargetPose(new Pose2d(12.42, 5.05, Rotation2d.fromDegrees(117.0)), 0.0);
    }

    public CCommand faceAprilTag(DoubleSupplier translationX, DoubleSupplier translationY) {
        // Positive TX means tag is to the right of the robot
        return this.cCommand("DriveSubsysem.DefaultDrive").onExecute(() -> {
            // Make the robot move
            swerveDrive.drive(
                    new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                    vision.getTx().orElse(0.0) * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }

    public Command sysIdDriveCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                new Config(), this, this.swerveDrive, 12, false),
                3.0,5.0,3.0
        );
    }
}
