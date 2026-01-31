package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Robot;
import frc.robot.Configuration.DriveSubsystemConfiguration;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import frc.robot.Subsystems.Vision.RealVision;
import frc.robot.Subsystems.Vision.SimVision;
import frc.robot.Subsystems.Vision.Vision;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Drive subsystem that controls the swerve drive system.
 * Manages robot movement, odometry, vision integration, and autonomous pathfinding.
 */
public class Drive extends CSubsystem {
    /** Maximum speed of the robot in meters per second. */
    // TODO: Configure robot details
    private static final double maxSpeed = Units.feetToMeters(6);
    /** The swerve drive object that manages the swerve modules. */
    private SwerveDrive swerveDrive;
    /** Vision subsystem for processing camera data and vision measurements. */
    private Vision vision;

    /**
     * Constructs a new Drive subsystem.
     * Initializes vision (real or simulated based on robot mode) and configures swerve drive.
     */
    public Drive() {

        if (Robot.isReal()) {
            vision = new RealVision(this);
        } else {
            vision = new SimVision(this);
        }

        // Temp starting positions for sim
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                Meter.of(4)),
                Rotation2d.fromDegrees(180))
                : new Pose2d(new Translation2d(Meter.of(16),
                        Meter.of(4)),
                        Rotation2d.fromDegrees(0));

        configureSwerveObjects(startingPose);
        DriveSubsystemConfiguration.configurePathPlanner(this, swerveDrive);
    }

    /**
     * Configures and initializes the swerve drive objects from JSON configuration files.
     * 
     * @param startingPose the initial pose of the robot on the field
     * @throws RuntimeException if the swerve configuration files cannot be loaded
     */
    public void configureSwerveObjects(Pose2d startingPose) {
        // Initialize the swerve drive object with the configs in the deploy directory
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Drive.maxSpeed, startingPose);
        } catch (IOException e) {
            System.out.println("Failed to load necessary files for swerve drive.");
            throw new RuntimeException(e);
        }

        swerveDrive.resetOdometry(startingPose);
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }

    /**
     * Gets the vision subsystem.
     * 
     * @return the vision subsystem instance
     */
    public Vision getVision() {
        return vision;
    }

    /**
     * Gets the swerve drive object.
     * 
     * @return the swerve drive instance
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
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
            swerveDrive.driveFieldOriented(
                    swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                            headingX.getAsDouble(),
                            headingY.getAsDouble(),
                            swerveDrive.getOdometryHeading().getRadians(),
                            swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return the current pose from odometry
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Resets the odometry to a specified pose.
     * 
     * @param pose2d the new pose to reset to
     */
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
        return cCommand("DriveSubsysem.DefaultDrive").onExecute(() -> {
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
        return driveCommand(translationX, translationY, angularRotationX)
                .onInitialize(() -> {
                    swerveDrive.setMaximumAllowableSpeeds(
                            swerveDrive.getMaximumChassisVelocity() / 2,
                            swerveDrive.getMaximumChassisAngularVelocity());
                }).onEnd(() -> {
                    swerveDrive.setMaximumAllowableSpeeds(
                            swerveDrive.getMaximumChassisVelocity() / 2,
                            swerveDrive.getMaximumChassisAngularVelocity());
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
            swerveDrive.getMaximumChassisVelocity(),      // Max velocity (m/s)
            4.0,                                                // Max acceleration (m/s^2)
            swerveDrive.getMaximumChassisAngularVelocity(), // Max angular velocity (rad/s)
            Math.PI * 4.0                                       // Max angular acceleration (rad/s^2)
        );
        
        // return AutoBuilder.pathfindToPose(
        //     targetPose,
        //     constraints,
        //     endVelocity  // Goal velocity at end
        // );
        return cCommand();
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
            swerveDrive.getMaximumChassisVelocity(),
            4.0,
            swerveDrive.getMaximumChassisAngularVelocity(),
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

    /**
     * Creates a command that drives to a dummy test pose.
     * 
     * @return command that drives to position (12.42, 5.05) with 117 degree rotation
     */
    public Command dummyDrivePose() {
        return driveToTargetPose(new Pose2d(12.42, 5.05, Rotation2d.fromDegrees(117.0)), 0.0);
    }

    // Find our alliance's HUB and the location
    public Translation2d getHub() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Translation2d allianceHub = new Translation2d(0,0);
        if(alliance.isPresent()) {
            if(alliance.get() == Alliance.Blue) {
                allianceHub = new Translation2d(182.11, 158.84);
            }
            if(alliance.get() == Alliance.Red) {
                allianceHub = new Translation2d(469.11, 158.84);
            }
        }

        return allianceHub;
    }

    // Calculate the distance that the robot is from our alliance's HUB
    public double hubDistance() {
        Translation2d allianceHub = getHub();
        Pose2d currentPose = getPose();

        double xDistance = Math.abs(currentPose.getX() - allianceHub.getX());
        double yDistance = Math.abs(currentPose.getY() - allianceHub.getY());

        // return the horizontal distance of the robot from the HUB in feet
        return Math.sqrt((Math.pow(xDistance, 2) + Math.pow(yDistance, 2))) / 12;
    }

    /**
     * Command to drive while automatically rotating to face an AprilTag.
     * Uses vision TX (horizontal angle) to calculate rotation.
     * 
     * @param translationX translation speed in the X direction
     * @param translationY translation speed in the Y direction
     * @return command that drives and rotates to face the target
     */
    public CCommand faceAprilTag(DoubleSupplier translationX, DoubleSupplier translationY) {
        // Positive TX means tag is to the right of the robot
        return cCommand("DriveSubsysem.DefaultDrive").onExecute(() -> {
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

    /**
     * Creates a SysId characterization command for the drive system.
     * Used to determine motor constants and system characteristics.
     * 
     * @return SysId routine command for drive characterization
     */
    public Command sysIdDriveCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                new Config(), this, this.swerveDrive, 12, true),
                3.0,5.0,3.0
        );
    }
}
