package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.Configuration.DriveSubsystemConfiguration;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ControllerConstants;
import frc.robot.Constants.RobotConstants.DriveSubsystemConstants;
import frc.robot.Constants.RobotConstants.FieldConstants;
import frc.robot.Constants.RobotConstants.ShooterSubsystemConstants;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import frc.robot.Libs.ShootOnTheMove;
import frc.robot.Subsystems.Vision.RealVision;
import frc.robot.Subsystems.Vision.Vision;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Drive subsystem that controls the swerve drive system.
 * Manages robot movement, odometry, vision integration, and autonomous pathfinding.
 */
public class Drive extends CSubsystem {
    /** Maximum speed of the robot in meters per second. */
    // TODO: Configure robot details
    private static final double maxSpeed = 5.5;
    /** The swerve drive object that manages the swerve modules. */
    private SwerveDrive swerveDrive;
    /** Vision subsystem for processing camera data and vision measurements. */
    private Vision vision;
    // Height to render poses in meters (matches shooter muzzle height used by sim)
    private static final double POSE_RENDER_Z = 0.9;

    private static Pose2d autoTrenchTarget = new Pose2d();  // Updated by getClosestPointOnCurve for dashboard display

    private final SwerveInputStream baseStream;
    private final CommandXboxController driverController;
    private final CommandXboxController codriverController;

    private final DoubleSupplier getTranslationX;
    private final DoubleSupplier getTranslationY;
    private final DoubleSupplier getHeadingX;
    private final DoubleSupplier getHeadingY;
    /**
     * Constructs a new Drive subsystem.
     * Initializes vision (real or simulated based on robot mode) and configures swerve drive.
     */
    public Drive(CommandXboxController driverController, CommandXboxController codriverController) {
        setName("DriveSubsystem");

        this.driverController = driverController;
        this.codriverController = codriverController;

        getTranslationX = () -> -1 * MathUtil.applyDeadband(this.driverController.getLeftY(),
            ControllerConstants.deadbandX);
        getTranslationY = () -> -1 * MathUtil.applyDeadband(this.driverController.getLeftX(),
            ControllerConstants.deadbandY);
        getHeadingX = () -> -1 * driverController.getRightX();
        getHeadingY = () -> -1 * driverController.getRightY();

        if (Robot.isReal()) {
            vision = new RealVision(this);
        } else {
        }

        // Temp starting positions for sim — use safe Dashboard.getAlliance() which provides a fallback
        boolean blueAlliance = Dashboard.getAlliance() == Alliance.Blue;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(3),
                Meter.of(4)),
                Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(Meter.of(16),
                        Meter.of(4)),
                        Rotation2d.fromDegrees(180));

        configureSwerveObjects(startingPose);

        baseStream = SwerveInputStream.of(swerveDrive, getTranslationX, getTranslationY)
            .allianceRelativeControl(true)
            .deadband(0.12);

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
     * Gets the current pose of the robot.
     * 
     * @return the current pose from odometry
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Returns true if the robot is on our alliance's half of the field.
     *
     * The field midline is computed as the midpoint between the blue and red hub
     * X coordinates. For the Blue alliance the 'our half' is the lower-X side;
     * for the Red alliance it's the higher-X side.
     *
     * @return true if the robot's X position is on our alliance half, false otherwise
     */
    public boolean isOnOurSide() {
        // Use hub X positions as the boundary. The field is effectively split in thirds
        // for our purposes: if we're Blue, we're 'on our side' when our X is less than
        // the Blue hub's X; if Red, when our X is greater than the Red hub's X.
        Pose2d pose = getPose();
        double x = pose.getX();

        // Use safe Dashboard.getAlliance() which will always return a value (with a sensible default)
        DriverStation.Alliance alliance = Dashboard.getAlliance();
        if (alliance == DriverStation.Alliance.Blue) {
            return x < FieldConstants.BLUE_HUB_POSITION.getX();
        } else {
            return x > FieldConstants.RED_HUB_POSITION.getX();
        }
    }

    /**
     * Returns true if autodrive (drive-to-pose) is active and the robot is within the given
     * translational and rotational tolerances of the target pose used by the autodrive stream.
     *
     * <p>For the built-in "drive-to-curve" autodrive this computes the target using the current
     * left-stick X input and the same snapping logic as {@link #getClosestPointOnCurve(double)}.
     *
     * @param posToleranceMeters translational tolerance in meters
     * @param rotToleranceRad rotational tolerance in radians
     * @return true if autodrive is active and robot is at the target pose
     */
    public boolean isAtAutoDriveTarget(double posToleranceMeters, double rotToleranceRad) {
        // If autodrive isn't active, by definition we're not "at the target while autodrive is on".
        if (!isDriveToPoseActive()) {
            return false;
        }

        try {
            // For the common "drive-to-curve" autodrive the target is computed from the left stick X.
            Pose2d targetPose = getClosestPointOnCurve(driverController.getLeftX());
            Pose2d current = getPose();

            double dist = current.getTranslation().getDistance(targetPose.getTranslation());

            double desiredAngle = targetPose.getRotation().getRadians();
            double currentAngle = current.getRotation().getRadians();
            double angleError = Math.atan2(Math.sin(desiredAngle - currentAngle), Math.cos(desiredAngle - currentAngle));

            boolean within = (dist <= posToleranceMeters) && (Math.abs(angleError) <= rotToleranceRad);

            SmartDashboard.putBoolean("Drive/AutoDrive/AtTarget", within);
            SmartDashboard.putNumber("Drive/AutoDrive/TargetDistMeters", dist);
            SmartDashboard.putNumber("Drive/AutoDrive/AngleErrorRad", angleError);

            return within;
        } catch (Exception ex) {
            return false;
        }
    }

    /**
     * Convenience overload that uses reasonable defaults for positional and rotational tolerances.
     * @return true if autodrive is active and the robot is at the target pose
     */
    public boolean isAtAutoDriveTarget() {
        // Default: 0.25 m position tolerance, 10 degrees rotation tolerance
        return isAtAutoDriveTarget(0.25, Math.toRadians(10.0));
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
        Rotation2d correctOffset = Dashboard.getAlliance() == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg;
        return cCommand("DriveSubsystem.ResetOdom").onInitialize(() -> {
            Pose2d newPose = new Pose2d( swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), correctOffset );
            swerveDrive.resetOdometry(newPose);
            SmartDashboard.putString("Drive/Odometry Reset", "Odometry reset to (0, 0, 0)");
        });
    }

    /**
     * Creates a command that drives to a climb position.
     * 
     * @return command that drives to closest climb position
     */

    // Calculate the distance that the robot is from our alliance's HUB
    public double hubDistance() {
        Translation2d allianceHub = FieldConstants.hub.get().getTranslation();
        Pose2d currentPose = getPose();

        double xDistance = Math.abs(currentPose.getX() - allianceHub.getX());
        double yDistance = Math.abs(currentPose.getY() - allianceHub.getY());

        // return the horizontal distance of the robot from the HUB in feet
        return Math.sqrt((Math.pow(xDistance, 2) + Math.pow(yDistance, 2))) / 12;
    }

    // Calculate the angular distance in degree that the robot is facing from our alliance's HUB
    public double hubAngle() {
        Translation2d allianceHub = FieldConstants.hub.get().getTranslation();
        Pose2d currentPose = getPose();

        double xDistance = Math.abs(currentPose.getX() - allianceHub.getX());
        double yDistance = Math.abs(currentPose.getY() - allianceHub.getY());
        
        return Math.toDegrees(Math.PI - (((Math.PI / 2) - currentPose.getRotation().getRadians()) + ((Math.PI / 2) - Math.atan( yDistance / xDistance ))));
    }

    /**
     * Return the scaled point using the robot's current translation from odometry.
     * Uses (x0,y0) = robot translation and returns (d / sqrt(x0^2 + y0^2)) * (x0,y0).
     * This keeps the function focused on the math you requested.
     */
    public Pose2d getClosestPointOnCurve(double controllerOffset) {
        // p = robot position, h = hub position, d = desired distance (midRange)
        Translation2d p = getPose().getTranslation();
        Translation2d h = FieldConstants.hub.get().getTranslation();

        SmartDashboard.putNumber("ControllerOffset", controllerOffset);

        // vector from hub to robot: v = p - h
        Translation2d v = p.minus(h);

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

        // Use the controller offset to slide left/right along the hub arc.
        // Map controllerOffset (-1..1) to an angular offset around the hub. We choose
        // a max angular excursion of +/- 90 degrees (pi/2) so the driver can move left/right
        // along the arc without flipping to the far side.
        double baseAngle = Math.atan2(v.getY(), v.getX()); // angle from hub->robot
        double maxAngle = Math.PI / 2.0; // +/- 90 degrees
        double angleOffset = MathUtil.clamp(controllerOffset, -1.0, 1.0) * maxAngle;
        double chosenTheta = baseAngle + angleOffset;

        // Compensate the aiming angle slightly opposite the robot's horizontal travel to
        // counteract momentum. Compute lateral sign relative to the hub->robot vector and
        // scale compensation by current speed fraction (0..1) with a small max angle.
        try {
            var vel = swerveDrive.getFieldVelocity();
            double vx = vel.vxMetersPerSecond;
            double vy = vel.vyMetersPerSecond;
            double travelSpeed = Math.hypot(vx, vy);
            double travelAngle = Math.atan2(vy, vx);

            // relative angle from hub->robot to travel vector
            double rel = travelAngle - baseAngle;
            // sign of lateral component (sin(rel) > 0 means movement is 'left' of the hub->robot line)
            double lateralSign = Math.signum(Math.sin(rel));

            // scale: fraction of max chassis speed (guard positive)
            double speedFraction = 0.0;
            double maxSpeed = swerveDrive.getMaximumChassisVelocity();
            if (maxSpeed > 1e-6) speedFraction = MathUtil.clamp(travelSpeed / maxSpeed, 0.0, 1.0);

            // make compensation stronger when driver offsets further from center: scale by |controllerOffset|
            double offsetFactor = Math.abs(MathUtil.clamp(controllerOffset, -1.0, 1.0));
            double offsetGain = 6.0; // stronger multiplier to make offset influence very noticeable

            // maximum compensation angle (radians) at full speed and full offset
            double maxCompensation = Math.toRadians(40.0); // substantially larger ( ~40 degrees )

            // scale compensation proportionally with lateral motion magnitude so straighter
            // forward/back motion doesn't add huge compensation. Use lateral magnitude (0..1).
            double lateralMagnitude = Math.abs(Math.sin(rel));

            // Compose final compensation: sign * speed fraction * offset factor * lateral magnitude * gain * max
            double compensation = -Math.signum(Math.sin(rel)) * speedFraction * offsetFactor * lateralMagnitude * offsetGain * maxCompensation;
            chosenTheta += compensation;

            SmartDashboard.putNumber("Drive/TravelSpeed", travelSpeed);
            SmartDashboard.putNumber("Drive/TravelAngleRad", travelAngle);
            SmartDashboard.putNumber("Drive/AngleCompensationRad", compensation);
            SmartDashboard.putNumber("Drive/OffsetFactor", offsetFactor);
            SmartDashboard.putNumber("Drive/OffsetGain", offsetGain);
        } catch (Exception ex) {
            // if anything goes wrong (e.g., swerveDrive not ready), skip compensation
        }

        Translation2d snappedTarget = new Translation2d(
            h.getX() + desiredDistanceMeters * Math.cos(chosenTheta),
            h.getY() + desiredDistanceMeters * Math.sin(chosenTheta)
        );

        double snappedTargetDist = Math.hypot(snappedTarget.getX() - h.getX(), snappedTarget.getY() - h.getY());
        SmartDashboard.putNumber("Drive/SnappedTargetDistanceFromHubMeters", snappedTargetDist);
        SmartDashboard.putNumber("Drive/DesiredDistanceMeters", desiredDistanceMeters);
        SmartDashboard.putNumber("Drive/TargetDistanceErrorMeters", snappedTargetDist - desiredDistanceMeters);
        SmartDashboard.putNumber("Drive/HubArcChosenAngleRad", chosenTheta);
        SmartDashboard.putNumber("Drive/ControllerOffset", controllerOffset);

        return posePointingAtAllianceHub(snappedTarget);
    }

    public Pose2d posePointingAtAllianceHub(Translation2d position) {
        Translation2d hubPosition = frc.robot.Constants.RobotConstants.FieldConstants.hub.get().getTranslation();

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

    /**
     * Builds a SwerveInputStream that aims the robot at the alliance hub while allowing the driver to control the heading with the right stick.
     * @return SwerveInputStream that aims at the hub and allows driver control of heading with right stick
     */
    public SwerveInputStream buildAimingStream() {

        DoubleSupplier getHeadingXSafe = () -> ShootOnTheMove.heading.isPresent() ? ShootOnTheMove.heading.get().getCos() : 0.0;
        DoubleSupplier getHeadingYSafe = () -> ShootOnTheMove.heading.isPresent() ? ShootOnTheMove.heading.get().getSin() : 0.0;

        return baseStream.copy()
            .withControllerHeadingAxis(getHeadingXSafe, getHeadingYSafe)
            .headingWhile(ShootOnTheMove.heading.isPresent())
            // Fallback
            .aim(FieldConstants.hub.get())
            .aimWhile(ShootOnTheMove.heading.isEmpty())
            .aimHeadingOffset(Rotation2d.k180deg)
            .aimHeadingOffset(ShootOnTheMove.heading.isEmpty());
    }

    /**
     * Builds a SwerveInputStream that allows the driver to control translation with the left stick and heading with the right stick.
     * @return SwerveInputStream that allows driver control of translation with left stick and heading with right stick
     */
    public SwerveInputStream buildDefaultStream() {
        return baseStream.copy()
            .withControllerHeadingAxis(getHeadingX, getHeadingY)
            .headingWhile(true);
    }

    /**
     * Builds a SwerveInputStream that uses the left stick X position to determine a point on the curve around the hub and drives to it using PathPlanner's pathfinding.
     * @return SwerveInputStream that drives to a point on the curve around the hub based on left stick X position
     */
    public SwerveInputStream buildDriveToCurveStream() {
        return baseStream
        .copy()
        .driveToPose(
            // Use a deadbanded left-stick X so the driver can move left/right along the curve
            () -> {
                double raw = driverController.getLeftX();
                double offset = MathUtil.applyDeadband(raw, ControllerConstants.deadbandX) / 2;
                return getClosestPointOnCurve(offset);
            },
            RobotConstants.DriveSubsystemConstants.translationProfiledController, 
            RobotConstants.DriveSubsystemConstants.rotationProfiledController
        )
        .driveToPoseEnabled(true);
    }

    public SwerveInputStream buildRelativeTurningStream() {
        return baseStream
            .copy()
            .withControllerRotationAxis(() -> -1 * driverController.getRightX());
    }

    public SwerveInputStream buildHalfDriveStream() {
        return baseStream
            .copy()
            .scaleTranslation(0.5)
            .scaleRotation(1)
            .withControllerHeadingAxis(getHeadingX, getHeadingY)
            .headingWhile(true);
    }

    /** 
     * Builds a {@link SwerveInputStream}
     */
    public SwerveInputStream buildTrenchStream() {
        return SwerveInputStream.of(
                swerveDrive, 
                getTranslationX, 
                () -> {
                    Pose2d current = getPose();
                    Translation2d closestTrench = getClosestTrench();
                    SmartDashboard.putNumberArray( "ClosestTrench", new double[] { closestTrench.getMeasureX().in(Meters), closestTrench.getMeasureY().in(Meters), 0.0 });
                    
                    if (current.getTranslation().getDistance(closestTrench) > 0.8) {
                        // Do NormalDrive
                        return getTranslationY.getAsDouble();
                    }

                    double horizonalDistance = closestTrench.getY() - current.getY();
                    return Math.min( Math.max( -1.0, horizonalDistance / 0.4 ), 1.0 ); // Clamp between 0 and 1
                }
            )
            .allianceRelativeControl(true)
            .deadband(0.12)
            .withControllerRotationAxis(getHeadingX);
            // .withControllerHeadingAxis(getHeadingX, getHeadingY)
            // .headingWhile(true);
    }

    public SwerveInputStream genericDriveToPoseStream(Supplier<Pose2d> pose) {
        return baseStream
            .copy()
            .driveToPose(pose, DriveSubsystemConstants.translationProfiledController, DriveSubsystemConstants.rotationProfiledController)
            .driveToPoseEnabled(true);
    }

    public SwerveInputStream genericDriveToPoseStream(Supplier<Pose2d> pose, BooleanSupplier condition) {
        return baseStream
            .copy()
            .driveToPose(pose, DriveSubsystemConstants.translationProfiledController, DriveSubsystemConstants.rotationProfiledController)
            .driveToPoseEnabled(condition);
    }

    /**
     * Returns the position closes trench
     * @return
     */
    public Translation2d getClosestTrench() {
        Pose2d current = getPose();
        if ( Meters.of(current.getX()).in(Inches) > (651/2)) {
            if (Meters.of(current.getY()).in(Inches) > (317/2)) {
                return FieldConstants.RED_LEFT_TRENCH;
            }
            return FieldConstants.RED_RIGHT_TRENCH;
        }
        if (Meters.of(current.getY()).in(Inches) > (317/2)) {
            return FieldConstants.BLUE_LEFT_TRENCH;
        }
        return FieldConstants.BLUE_RIGHT_TRENCH;

    }

    public Pose2d getClosestElevator() {
        // Use safe Dashboard.getAlliance() — no optional handling required
        System.out.println( "Pos: " + getPose().getY() + " Elevator: " + Inches.of(317/2).in(Meters) );
        switch( Dashboard.getAlliance() ) {
            case Blue:
                if ( getPose().getY() > Inches.of(317/2).in(Meters) ) {
                    return FieldConstants.BLUE_LEFT_CLIMB_POSITION;
                }
                return FieldConstants.BLUE_RIGHT_CLIMB_POSITION;
            case Red:
                if ( getPose().getY() > Inches.of(317/2).in(Meters) ) {
                    return FieldConstants.RED_LEFT_CLIMB_POSITION;
                }
                return FieldConstants.RED_RIGHT_CLIMB_POSITION;
        }
        return getPose();
    }

    public SwerveInputStream buildDriveToElevator() {
        return genericDriveToPoseStream(() -> { 
            Pose2d closest = getClosestElevator();
            SmartDashboard.putNumberArray("BestElevator", new double[] {closest.getX(), closest.getY(), closest.getRotation().getDegrees()});
            return closest;
        });
    }

    public Pose2d poseFromTranslation(Translation2d location) {
        return new Pose2d(location, Rotation2d.kZero);
    }

    /**
     * Compute the appropriate trench target for the current pose.
     * If opponentSide is true, compute the target on the opponent's side (mirror alliances).
     */
    private Pose2d computeTrenchTarget(boolean opponentSide) {
        DriverStation.Alliance alliance = Dashboard.getAlliance();
        if (opponentSide) {
            alliance = (alliance == DriverStation.Alliance.Blue) ? DriverStation.Alliance.Red : DriverStation.Alliance.Blue;
        }

        if (alliance == DriverStation.Alliance.Blue) {
            if ( getPose().getY() > FieldConstants.yHalfLine ) {
                if ( getPose().getX() > FieldConstants.BLUE_LEFT_TRENCH.getX() ) {
                    return poseFromTranslation(FieldConstants.MID_2_BLUE_LEFT_TRENCH);
                }
                return poseFromTranslation(FieldConstants.BLUE_2_MID_LEFT_TRENCH);
            }
            if ( getPose().getX() > FieldConstants.BLUE_LEFT_TRENCH.getX() ) {
                return poseFromTranslation(FieldConstants.MID_2_BLUE_RIGHT_TRENCH);
            }
            return poseFromTranslation(FieldConstants.BLUE_2_MID_RIGHT_TRENCH);
        } else {
            if ( getPose().getY() > FieldConstants.yHalfLine ) {
                if ( getPose().getX() < FieldConstants.RED_LEFT_TRENCH.getX() ) {
                    return poseFromTranslation(FieldConstants.MID_2_RED_LEFT_TRENCH);
                }
                return poseFromTranslation(FieldConstants.RED_2_MID_LEFT_TRENCH);
            }
            if ( getPose().getX() < FieldConstants.RED_LEFT_TRENCH.getX() ) {
                return poseFromTranslation(FieldConstants.MID_2_RED_RIGHT_TRENCH);
            }
            return poseFromTranslation(FieldConstants.RED_2_MID_RIGHT_TRENCH);
        }
    }

    /**
     * drive through trench on your alliance side
     * @return
     */
    public Command driveThroughTrenchSS() {
        Supplier<Pose2d> getTargetLocation = () -> computeTrenchTarget(false);
        return driveToTargetPose(autoDriveTargetLogger(getTargetLocation), 0);
    }

    /**
     * drive through trench oponant side
     * @return
     */
    public Command driveThroughTrenchOS() {
        Supplier<Pose2d> getTargetLocation = () -> computeTrenchTarget(true);
        return driveToTargetPose(autoDriveTargetLogger(getTargetLocation), 0);
    }

    public Supplier<Pose2d> autoDriveTargetLogger( Supplier<Pose2d> targetGetter ) {
        return () -> {
            Supplier<Pose2d> supplier = targetGetter;
            Pose2d targetPose = supplier.get();
            SmartDashboard.putNumberArray("AutoDriveTarget", new double[] {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
            autoTrenchTarget = targetPose;
            return targetPose;
        };
    }

    @Override
    public void periodic() {
        logSelf();
        Dashboard.getField2d().setRobotPose(getPose());

        ShootOnTheMove.calculateLeadHeading(getPose(), swerveDrive.getRobotVelocity());
        ShootOnTheMove.publish();
    }
}
