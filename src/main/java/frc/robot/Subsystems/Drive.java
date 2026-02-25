package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Configuration.DriveSubsystemConfiguration;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.FieldConstants;
import frc.robot.Constants.RobotConstants.ShooterSubsystemConstants;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
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
    // Height to render poses in meters (matches shooter muzzle height used by sim)
    private static final double POSE_RENDER_Z = 0.9;

    /**
     * Constructs a new Drive subsystem.
     * Initializes vision (real or simulated based on robot mode) and configures swerve drive.
     */
    public Drive() {

        // if (Robot.isReal()) {
        //     vision = new RealVision(this);
        // } else {
        // }

        // Temp starting positions for sim
        boolean blueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                Meter.of(4)),
                Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(Meter.of(16),
                        Meter.of(4)),
                        Rotation2d.fromDegrees(180));

        configureSwerveObjects(startingPose);
        DriveSubsystemConfiguration.configurePathPlanner(this, swerveDrive);
    }

    // Flags set by RobotContainer when certain input streams are active
    private volatile boolean driveToPoseActive = false;
    private volatile boolean aimModeActive = false;

    /**
     * Mark whether the drive-to-pose input stream is currently active.
     * Called by RobotContainer when the corresponding input is pressed/released.
     */
    public void setDriveToPoseActive(boolean active) { this.driveToPoseActive = active; }
    public boolean isDriveToPoseActive() { return this.driveToPoseActive; }

    /**
     * Mark whether the aim input stream (aim mode) is currently active.
     */
    public void setAimModeActive(boolean active) { this.aimModeActive = active; }
    public boolean isAimModeActive() { return this.aimModeActive; }

    /**
     * Check whether the robot is aimed at the alliance hub within a tolerance (radians).
     */
    public boolean isAimedAtHub(double tolRad) {
        try {
            Pose2d desired = posePointingAtAllianceHub(getPose().getTranslation());
            double desiredAngle = desired.getRotation().getRadians();
            double currentAngle = getPose().getRotation().getRadians();
            double angleError = Math.atan2(Math.sin(desiredAngle - currentAngle), Math.cos(desiredAngle - currentAngle));
            return Math.abs(angleError) <= tolRad;
        } catch (Exception ex) {
            return false;
        }
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
     * Takes in a pose from vision and uses it to update the swerve odometry.
     * This method should be called with the proper timestamp from the vision system.
     *
     * @param pose Current pose of the robot from vision
     * @param timestampSeconds The timestamp when the pose was captured (in seconds)
     */
    public void updatePose(Pose2d pose, double timestampSeconds) {
        swerveDrive.addVisionMeasurement(pose, timestampSeconds);
    }

    /**
     * Command to reset odometry using the current Limelight pose.
     * Only resets if a valid vision pose is available.
     * 
     * @return Command that resets odometry to the current vision pose
     */
    public Command resetOdometryWithVision() {
        return runOnce(() -> {
            if (vision != null) {
                Optional<Pose2d> visionPose = vision.getRobotPose();
                if (visionPose.isPresent()) {
                    resetOdometry(visionPose.get());
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Drive/Odometry Reset", 
                        "Reset to vision pose: " + visionPose.get().toString());
                } else {
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Drive/Odometry Reset", 
                        "No valid vision pose available");
                }
            }
        });
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
     * Build a command that drives the robot using a supplier of ChassisSpeeds.
     * The supplier should provide desired chassis velocities in the robot frame
     * (vx, vy in m/s and omega in rad/s). This method creates a command that
     * reads the supplier each loop and forwards the speeds to the swerve drive.
     *
     * @param speedsSupplier Supplier that returns desired ChassisSpeeds
     * @return CCommand that drives with the supplied chassis speeds
     */
    public CCommand driveWithChassisSpeedsSupplier(java.util.function.Supplier<ChassisSpeeds> speedsSupplier) {
        return cCommand("DriveSubsystem.DriveWithChassisSpeeds").onExecute(() -> {
            ChassisSpeeds speeds = speedsSupplier.get();
            // Convert chassis speeds to translation (vx, vy) in m/s and omega in rad/s
            Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            double omega = speeds.omegaRadiansPerSecond;
            // Telemetry to help debug aiming behavior
            try {
                Pose2d desiredPose = posePointingAtAllianceHub(getPose().getTranslation());
                double desiredAngle = desiredPose.getRotation().getRadians();
                double currentAngle = getPose().getRotation().getRadians();
                double angleError = Math.atan2(Math.sin(desiredAngle - currentAngle), Math.cos(desiredAngle - currentAngle));
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/ChassisSpeeds/Vx", speeds.vxMetersPerSecond);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/ChassisSpeeds/Vy", speeds.vyMetersPerSecond);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/ChassisSpeeds/Omega", omega);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/DesiredAngleRad", desiredAngle);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/CurrentAngleRad", currentAngle);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/AngleErrorRad", angleError);
            } catch (Exception e) {
                // ignore telemetry errors
            }
            // Drive the swerve with given speeds (robot-relative)
            swerveDrive.drive(translation, omega, true, false);
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
            Math.PI * 11                                       // Max angular acceleration (rad/s^2)
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

    public CCommand resetOdom() {
        return cCommand("DriveSubsystem.ResetOdom").onInitialize(() -> {
            Pose2d newPose = new Pose2d( swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), Rotation2d.fromDegrees(0) );
            swerveDrive.resetOdometry(newPose);
            SmartDashboard.putString("Drive/Odometry Reset", "Odometry reset to (0, 0, 0)");
        });
    }

    /**
     * Creates a command that drives to a dummy test pose.
     * 
     * @return command that drives to position (10.3, 4.3) with 10 degree rotation
     */
    public Command dummyDrivePose() {
        return driveToTargetPose(new Pose2d(10.3, 4.3, Rotation2d.fromDegrees(10)), 0.0)
            .beforeStarting(() -> {
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("PathPlanner/Status", "Starting pathfind");
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("PathPlanner/Target", 
                    String.format("(%.2f, %.2f, %.1f°)", 10.3, 4.3, 10.0));
            })
            .andThen(() -> {
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("PathPlanner/Status", "Pathfind complete");
            });
    }

    /**
     * Creates a command that drives to a climb position.
     * 
     * @return command that drives to closest climb position
     */
    public Command driveToClimb() {
        Pose2d targetPose;
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            // Choose the closest blue climb position
            Pose2d blueRight = RobotConstants.FieldConstants.BLUE_RIGHT_CLIMB_POSITION;
            Pose2d blueLeft = RobotConstants.FieldConstants.BLUE_LEFT_CLIMB_POSITION;
            targetPose = (getPose().getTranslation().getDistance(blueRight.getTranslation()) < 
                          getPose().getTranslation().getDistance(blueLeft.getTranslation())) ? blueRight : blueLeft;
        } else {
            // Choose the closest red climb position
            Pose2d redRight = RobotConstants.FieldConstants.RED_RIGHT_CLIMB_POSITION;
            Pose2d redLeft = RobotConstants.FieldConstants.RED_LEFT_CLIMB_POSITION;
            targetPose = (getPose().getTranslation().getDistance(redRight.getTranslation()) < 
                          getPose().getTranslation().getDistance(redLeft.getTranslation())) ? redRight : redLeft;
        }

        return driveToTargetPose(targetPose, 0.0)
            .beforeStarting(() -> {
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("PathPlanner/Status", "Starting pathfind to climb");
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("PathPlanner/Target", 
                    String.format("(%.2f, %.2f, %.1f°)", targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()));
            })
            .andThen(() -> {
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("PathPlanner/Status", "Pathfind to climb complete");
            });
    }

    // Find our alliance's HUB and the location
    public Translation2d getHub() {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                return RobotConstants.FieldConstants.BLUE_HUB_POSITION;
            } else {
                return RobotConstants.FieldConstants.RED_HUB_POSITION;
            }
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

    // Calculate the angular distance in degree that the robot is facing from our alliance's HUB
    public double hubAngle() {
        Translation2d allianceHub = getHub();
        Pose2d currentPose = getPose();

        double xDistance = Math.abs(currentPose.getX() - allianceHub.getX());
        double yDistance = Math.abs(currentPose.getY() - allianceHub.getY());
        
        return Math.toDegrees(Math.PI - (((Math.PI / 2) - currentPose.getRotation().getRadians()) + ((Math.PI / 2) - Math.atan( yDistance / xDistance ))));
    }

    /**
     * Command to drive while automatically rotating to face an AprilTag.
     * Uses hubAngle to calculate rotation.
     * 
     * @param translationX translation speed in the X direction
     * @param translationY translation speed in the Y direction
     * @return command that drives and rotates to face the target
     */
    public CCommand faceHub(DoubleSupplier translationX, DoubleSupplier translationY) {
        return cCommand( "DriveSubsysem.DefaultDrive").onExecute(() -> {
            //Make the robot move
            swerveDrive.drive(
                new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                hubAngle() * swerveDrive.getMaximumChassisAngularVelocity(),
                true,
                false);
        });
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
                new Config(), this, this.swerveDrive, 12, false),
                3.0,5.0,3.0
        );
    }

    /**
     * Periodic method called every robot loop (20ms).
     * Vision subsystem has its own periodic method that runs automatically via CommandScheduler.
     */
    @Override
    public void periodic() {
        // Vision subsystem periodic is called automatically by CommandScheduler
        // No need to manually call it here
        // Publish visual helpers (hub arc) so AdvantageScope / dashboard can render them
        publishHubArc();
    }

    /**
     * Publish a polyline (x,y pairs) describing the circular arc around the hub
     * at the configured shooter mid-range. AdvantageScope can render this by
     * reading the NetworkTables array at key "Drive/HubArcPointsMeters".
     *
     * Published format: [x0, y0, x1, y1, x2, y2, ...] in meters (field coordinates).
     */
    private void publishHubArc() {
        final int points = 64; // resolution of the circle
        Translation2d h = FieldConstants.hub.getTranslation();
        double r = ShooterSubsystemConstants.midRange; // midRange is in meters

        double[] pts = new double[points * 2];
        for (int i = 0; i < points; ++i) {
            double theta = 2.0 * Math.PI * ((double) i / (double) points);
            pts[i * 2] = h.getX() + r * Math.cos(theta);
            pts[i * 2 + 1] = h.getY() + r * Math.sin(theta);
        }

        SmartDashboard.putNumberArray("Drive/HubArcPointsMeters", pts);
    }

    /**
     * Return the scaled point using the robot's current translation from odometry.
     * Uses (x0,y0) = robot translation and returns (d / sqrt(x0^2 + y0^2)) * (x0,y0).
     * This keeps the function focused on the math you requested.
     */
    public Pose2d getClosestPointOnCurve(double controllerOffset) {
        // p = robot position, h = hub position, d = desired distance (midRange)
        Translation2d p = getPose().getTranslation();
        Translation2d h = FieldConstants.hub.getTranslation();

        SmartDashboard.putNumber("ControllerOffset", controllerOffset);

        // vector from hub to robot: v = p - h
        Translation2d v = p.minus(h);
        if ( Math.abs(controllerOffset) > 0.1 ) {
            Translation2d offset = new Translation2d( 0, -1);
            v.plus(offset);
        }

        // distance ||v||
        double dist = Math.hypot(v.getX(), v.getY());

        // Guard against division by zero (robot exactly at hub)
        if (dist < 1e-6) {
            // undefined direction; return a pose at the current position pointing at the hub
            return posePointingAtAllianceHub(p);
        }

        // ShooterSubsystemConstants.midRange is in meters (use directly)
        double desiredDistanceMeters = ShooterSubsystemConstants.midRange;

        // factor = d / ||v||
        double factor = desiredDistanceMeters / dist;

        // target = h + factor * v  -> matches: d/||p-h|| * (p-h) + (h_x,h_y)
        Translation2d continuousTarget = v.times(factor).plus(h);

        // Publish continuous target distance to hub (should be equal to desiredDistanceMeters)
        double continuousTargetDist = Math.hypot(continuousTarget.getX() - h.getX(), continuousTarget.getY() - h.getY());
        SmartDashboard.putNumber("Drive/ContinuousTargetDistanceFromHubMeters", continuousTargetDist);

        // Snap to nearest vertex on the published hub arc so the logged target matches the visual arc
        final int arcPoints = 64;
        int nearestIndex = 0;
        double nearestDistSq = Double.POSITIVE_INFINITY;
        Translation2d snappedTarget = continuousTarget;
        for (int i = 0; i < arcPoints; ++i) {
            double theta = 2.0 * Math.PI * ((double) i / (double) arcPoints);
            double px = h.getX() + desiredDistanceMeters * Math.cos(theta);
            double py = h.getY() + desiredDistanceMeters * Math.sin(theta);
            double dx = continuousTarget.getX() - px;
            double dy = continuousTarget.getY() - py;
            double d2 = dx * dx + dy * dy;
            if (d2 < nearestDistSq) {
                nearestDistSq = d2;
                nearestIndex = i;
                snappedTarget = new Translation2d(px, py);
            }
        }

        double snappedTargetDist = Math.hypot(snappedTarget.getX() - h.getX(), snappedTarget.getY() - h.getY());
        SmartDashboard.putNumber("Drive/SnappedTargetDistanceFromHubMeters", snappedTargetDist);
        SmartDashboard.putNumber("Drive/DesiredDistanceMeters", desiredDistanceMeters);
        SmartDashboard.putNumber("Drive/TargetDistanceErrorMeters", snappedTargetDist - desiredDistanceMeters);
        SmartDashboard.putNumber("Drive/HubArcNearestIndex", nearestIndex);

        return posePointingAtAllianceHub(snappedTarget);
    }

    public Pose2d posePointingAtAllianceHub(Translation2d position) {
        Translation2d hubPosition = frc.robot.Constants.RobotConstants.FieldConstants.hub.getTranslation();

    // Publish hub location for debugging as Pose3d: [x, y, z, rollDeg, pitchDeg, yawDeg]
    SmartDashboard.putNumberArray("Drive/HubPose", new double[] { hubPosition.getX(), hubPosition.getY(), POSE_RENDER_Z, 0.0, 0.0, 0.0 });

        // Angle from the given position to the hub (robot should point at the hub)
        double angle = Math.atan2(hubPosition.getY() - position.getY(), hubPosition.getX() - position.getX());

        // Create a pose at the provided position with rotation toward the hub.
        // IMPORTANT: do NOT add the hub translation again; that would produce position + hub which is incorrect.
        Pose2d targetPose = new Pose2d(position, new Rotation2d(angle).rotateBy(Rotation2d.k180deg));

        // Prepare the arc pose3d for publishing (default)
        Pose3d arcPose3d = new Pose3d(new Translation3d(targetPose.getX(), targetPose.getY(), POSE_RENDER_Z), new Rotation3d(0.0, 0.0, targetPose.getRotation().getRadians()));
        SmartDashboard.putNumberArray("Drive/TargetPoseArc", new double[] { arcPose3d.getX(), arcPose3d.getY(), arcPose3d.getZ(), Math.toDegrees(arcPose3d.getRotation().getX()), Math.toDegrees(arcPose3d.getRotation().getY()), Math.toDegrees(arcPose3d.getRotation().getZ()) });

        // Try to find a projectile close to the snapped target; if present, publish the ball Pose3d as the target
        Pose3d chosenPose3d = arcPose3d;

        // Publish the chosen target pose (ball if present, otherwise arc)
        SmartDashboard.putNumberArray("Drive/TargetPose", new double[] { chosenPose3d.getX(), chosenPose3d.getY(), chosenPose3d.getZ(), Math.toDegrees(chosenPose3d.getRotation().getX()), Math.toDegrees(chosenPose3d.getRotation().getY()), Math.toDegrees(chosenPose3d.getRotation().getZ()) });

        return targetPose;
    }
}
