package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import com.pathplanner.lib.pathfinding.Pathfinder;

import static edu.wpi.first.units.Units.*;

public class Drive extends CSubsystem {
    // TODO: Configure robot details
    private static final double maxSpeed = Units.feetToMeters(6);
    private SwerveDrive swerveDrive;
    private Vision vision;

    // PathPlanner Drive controller
    private final AutoBuilder autoBuilder = new AutoBuilder();

    // Vision Pid Controllers
    private final PIDController xController = new PIDController(5, 0, 0);
    private final PIDController yController = new PIDController(5, 0, 0);
    private ProfiledPIDController visionTurnController;
    private HolonomicDriveController visionDriveController;

    public Drive() {
        // For use when running a sim, disable otherwise
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        // Temp starting positions for sim
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                Meter.of(4)),
                Rotation2d.fromDegrees(180))
                : new Pose2d(new Translation2d(Meter.of(16),
                Meter.of(4)),
                Rotation2d.fromDegrees(0));

        this.configureSwerveObjects( startingPose );
        Configs.DriveSubsystem.configurePathPlanner( this, this.swerveDrive, this.autoBuilder );

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints( 6.0, 3.0 );
         this.visionTurnController = new ProfiledPIDController(5.0, 0.0, 0.0, constraints );
         this.visionDriveController = new HolonomicDriveController(
                this.xController,
                this.yController,
                this.visionTurnController
         );
    }

    public void configureSwerveObjects( Pose2d startingPose ) {
        // Initialize the swerve drive object with the configs in the deploy directory
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            this.swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Drive.maxSpeed, startingPose);
        } catch (IOException e) {
            System.out.println("Failed to load necessary files for swerve drive.");
            throw new RuntimeException(e);
        }

        this.swerveDrive.resetOdometry(startingPose);
    }



    public Vision getVision() {
        return this.vision;
    }

    public AutoBuilder getAutoBuilder() { return this.autoBuilder; }

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

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 3);
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
     * Takes in a pose from the limelight and uses it to update the swerve modules position.
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
        return this.driveCommand(translationX, translationY, angularRotationX)
                .onInitialize(() -> {
                    this.swerveDrive.setMaximumAllowableSpeeds(
                            this.swerveDrive.getMaximumChassisVelocity() / 2,
                            this.swerveDrive.getMaximumChassisAngularVelocity()
                    );
                }).onEnd(() -> {
                    this.swerveDrive.setMaximumAllowableSpeeds(
                            this.swerveDrive.getMaximumChassisVelocity() / 2,
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
    public ChassisSpeeds getSpeedsForTarget(Pose2d target) {
        return this.visionDriveController.calculate(
                this.swerveDrive.getPose(),
                target,
                this.swerveDrive.getMaximumChassisVelocity(),
                // 0,
                target.getRotation()
        );
    }

    public CCommand driveToTargetPose(Supplier<Pose2d> pose2dSupplier) {
        //PathConstraints constraints = new PathConstraints(
                //this.swerveDrive.getMaximumChassisVelocity(),
                //1.5,
                //this.swerveDrive.getMaximumChassisAngularVelocity() * Math.PI / 180,
                //Math.pow( this.swerveDrive.getMaximumChassisAngularVelocity() * Math.PI / 180, 2 )
        //);
        //this.autoBuilder.pathfindToPose(
                //pose2dSupplier.get(),
                //constraints,
                //0.0 // Target ending MPS, Goal is to not be moving.
        //);
        return this.cCommand("DriveSubsystem.DriveToTargetPoseCommand")
                .onExecute(() -> {
                    ChassisSpeeds targetSpeeds = this.getSpeedsForTarget(pose2dSupplier.get());
                    swerveDrive.drive(targetSpeeds);
                });
    }

    public CCommand dummyDrivePose() {
        return this.driveToTargetPose(() -> new Pose2d(12.42, 5.05, Rotation2d.fromDegrees(117.0)));
    }

    public CCommand faceAprilTag(DoubleSupplier translationX, DoubleSupplier translationY) {
        // Positive TX means tag is to the right of the robot
        return this.cCommand("DriveSubsysem.DefaultDrive").onExecute(() -> {
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
