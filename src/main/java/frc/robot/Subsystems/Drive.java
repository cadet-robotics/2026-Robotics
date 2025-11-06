package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drive extends CSubsystem {
    // TODO: Configure robot details
    private static final double maxSpeed = Units.feetToMeters(6);
    private final SwerveDrive swerveDrive;
    // Choreo Setup
    private final PIDController xController = new PIDController(10, 0, 0);
    private final PIDController yController = new PIDController(10, 0, 0);
    private final PIDController headingController = new PIDController(7.5, 0, 0);
    private final Vision vision;

    private final HolonomicDriveController visionDriveController;
    private final ProfiledPIDController visionTurnController = new ProfiledPIDController(
            7.5,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    Constants.DriveSubsystem.maxTurnSpeed,
                    Math.pow( Constants.DriveSubsystem.maxTurnSpeed, 2
            )));

    public Drive() {
        // For use when running a sim, disable otherwise
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                Meter.of(4)),
                Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(Meter.of(16),
                Meter.of(4)),
                Rotation2d.fromDegrees(180));
        // Initialize the swerve drive object with the configs in the deploy directory
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            this.swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Drive.maxSpeed, startingPose );
        } catch (IOException e) {
            System.out.println("Failed to load necessary files for swerve drive.");
            throw new RuntimeException(e);
        }

        this.vision = new Vision(this);

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.headingController.enableContinuousInput(0,360);
        this.visionDriveController = new HolonomicDriveController(
                this.xController,
                this.yController,
                this.visionTurnController
        );
    }

    public Vision getVision() {
        return this.vision;
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

            SmartDashboard.putBoolean("Sim Running", true );
            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
            // Make the robot move
            this.swerveDrive.driveFieldOriented(
                    swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                            headingX.getAsDouble(),
                            headingY.getAsDouble(),
                            swerveDrive.getOdometryHeading().getRadians(),
                            swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Command to follow a trajectory. Used in Choreo Autos.
     *
     * @param sample Choreo sample to follow.
     */
    public void followTrajectory( SwerveSample sample ) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x ),
                sample.vy + yController.calculate(pose.getY(), sample.y ),
                sample.omega + headingController.calculate( pose.getRotation().getRadians(), sample.heading )
        );

        // Apply the generated speeds
        swerveDrive.drive(speeds);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose2d) {
        swerveDrive.resetOdometry(pose2d);
    }

    /**
     * Takes in a pose from the limelight and uses it to update the swerve modules position.
     *
     * @param pose Current pose of the robot
     */
    public void updatePose(Pose2d pose ) {
        swerveDrive.addVisionMeasurement( pose, Timer.getFPGATimestamp() );
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
        return this.cCommand("DriveSubsysem.DefaultDrive").onExecute(()-> {
            // Make the robot move
           swerveDrive.drive(
            new Translation2d(
                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
            ),
            angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
            true,
            false
           );
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
        DoubleSupplier angularRotationX
    ) {
        return this.driveCommand(translationX, translationY, angularRotationX )
           .onInitialize(() -> {
              this.swerveDrive.setMaximumAllowableSpeeds(
                  Drive.maxSpeed / 2,
                  this.swerveDrive.getMaximumChassisAngularVelocity()
              );
           }).onEnd(() -> {
                    this.swerveDrive.setMaximumAllowableSpeeds(
                            Drive.maxSpeed,
                            this.swerveDrive.getMaximumChassisAngularVelocity()
                    );
           });
    }

    /**
     * Converts a target position into usable chassis speeds for the drivebase.
     *
     * @param target The target pose
     * @return
     */
    public ChassisSpeeds getSpeedsForTarget( Pose2d target ) {
        return this.visionDriveController.calculate(
            this.swerveDrive.getPose(),
            target,
            Constants.DriveSubsystem.maxSpeed,
            target.getRotation()
        );
    }

    public CCommand driveToTargetPose( Supplier<Pose2d> pose2dSupplier ) {
        return this.cCommand("DriveSubsystem.DriveToTargetPoseCommand")
            .onExecute(() -> {
                ChassisSpeeds targetSpeeds = this.getSpeedsForTarget(pose2dSupplier.get());
                swerveDrive.drive( targetSpeeds );
            });
    }

    public CCommand faceAprilTag( DoubleSupplier translationX, DoubleSupplier translationY ) {
        // Positive TX means tag is to the right of the robot
        return this.cCommand("DriveSubsysem.DefaultDrive").onExecute(()-> {
            // Make the robot move
            swerveDrive.drive(
                new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
                ),
        vision.getTx() * swerveDrive.getMaximumChassisAngularVelocity(),
    true,
    false
            );
        });
    }
}
