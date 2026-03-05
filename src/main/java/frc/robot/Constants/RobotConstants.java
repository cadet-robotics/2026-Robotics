package frc.robot.Constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Dashboard;

/**
 * Container class for all robot constants.
 * Constants are organized into nested classes by subsystem or component.
 */
public class RobotConstants {
    /**
     * Constants for Xbox controller configuration.
     * Defines deadband values to prevent stick drift and unintended movement.
     */
    public static final class ControllerConstants {
        /** Deadband threshold for X-axis joystick input (prevents stick drift). */
        public static final double deadbandX = 0.15;
        /** Deadband threshold for Y-axis joystick input (prevents stick drift). */
        public static final double deadbandY = 0.15;
    }

    /**
     * Constants for field positions and landmarks.
     * All positions are in inches from the origin.
     */
    public static final class FieldConstants {
        /** Blue alliance hub/speaker position (x, y) in inches. */
        public static final Translation2d BLUE_HUB_POSITION = new Translation2d(Inches.of(182.11).in(Meters), Inches.of(158.84).in(Meters) );
        /** Red alliance hub/speaker position (x, y) in inches. */
        public static final Translation2d RED_HUB_POSITION = new Translation2d( Inches.of(469.11).in(Meters), Inches.of(158.84).in(Meters) );

        public static Supplier<Pose2d> hub = () -> {
            return Dashboard.getAlliance() == Alliance.Blue
                ? new Pose2d(BLUE_HUB_POSITION, Rotation2d.fromDegrees(0))
                : new Pose2d(RED_HUB_POSITION, Rotation2d.fromDegrees(0));
        };
        
        /** Blue alliance rightside climb position (x, y, Rotation2d) in inches and degrees. */
        public static final Pose2d BLUE_RIGHT_CLIMB_POSITION = new Pose2d(Inches.of(37.6075), Inches.of(112.5065), Rotation2d.fromRadians(0));
        /** Blue alliance leftside climb position (x, y, Rotation2d) in inches and degrees. */
        public static final Pose2d BLUE_LEFT_CLIMB_POSITION = new Pose2d(Inches.of(42.3925), Inches.of(182.4335), Rotation2d.fromRadians(Math.PI));
        /** Red alliance rightside climb position (x, y, Rotation2d) in inches and degrees. */
        public static final Pose2d RED_RIGHT_CLIMB_POSITION = new Pose2d(Inches.of(608.8275), Inches.of(135.5065), Rotation2d.fromRadians(0));
        /** Red alliance leftside climb position (x, y, Rotation2d) in inches and degrees. */
        public static final Pose2d RED_LEFT_CLIMB_POSITION = new Pose2d(Inches.of(612.6125), Inches.of(208.4335), Rotation2d.fromRadians(Math.PI));

        // Center of Trench positions for blue and red alliance, 
        public static final Translation2d BLUE_LEFT_TRENCH = new Translation2d(Inches.of(182), Inches.of(317-25));
        public static final Translation2d BLUE_RIGHT_TRENCH = new Translation2d(Inches.of(182), Inches.of(25));
        public static final Translation2d RED_LEFT_TRENCH = new Translation2d(Inches.of(651-182), Inches.of(317-25));
        public static final Translation2d RED_RIGHT_TRENCH = new Translation2d(Inches.of(651-182), Inches.of(25));

        public static Translation2d offset = new Translation2d( 1.25, 0 );
        
        public static final Translation2d RED_RED_LEFT_TRENCH = RED_LEFT_TRENCH.plus(offset);
        public static final Translation2d RED_MID_LEFT_TRENCH = RED_LEFT_TRENCH.minus(offset);

        public static final Translation2d RED_RED_RIGHT_TRENCH = RED_RIGHT_TRENCH.plus(offset);
        public static final Translation2d RED_MID_RIGHT_TRENCH = RED_RIGHT_TRENCH.minus(offset);
        
        public static final Translation2d BLUE_BLUE_LEFT_TRENCH = BLUE_LEFT_TRENCH.minus(offset);
        public static final Translation2d BLUE_MID_LEFT_TRENCH = BLUE_LEFT_TRENCH.plus(offset);

        public static final Translation2d BLUE_BLUE_RIGHT_TRENCH = BLUE_RIGHT_TRENCH.minus(offset);
        public static final Translation2d BLUE_MID_RIGHT_TRENCH = BLUE_RIGHT_TRENCH.plus(offset);
    }

    /**
     * Constants for the swerve drive subsystem.
     * Includes maximum speeds and PID constants for translation and rotation control.
     */
    public static final class DriveSubsystemConstants {
        /** Maximum rotational speed of the robot in degrees per second. */
        public static final double maxTurnSpeed = 480; // Degrees
        /** Maximum translational speed of the robot in meters per second. */
        public static final double maxSpeed = 6; // Meters per second

        /** PID constants for translational (x, y) movement control. Pathplanner */
        public static final PIDConstants pptranslationPidConstants = new PIDConstants(20, 0, 1);
        /** PID constants for rotational (theta) movement control. Pathplanner */
        public static final PIDConstants pprotationPidConstants = new PIDConstants(6, 0, 0);

        /** PID constants for translational (x, y) movement control. */
        public static final PIDConstants translationPidConstants = new PIDConstants(4.5, 0, 1);
        /** PID constants for rotational (theta) movement control. */
        public static final PIDConstants rotationPidConstants = new PIDConstants(6, 0, 0);

        /** Motion profile constraints for translation (meters/sec, meters/sec^2) */
        public static final double TRANSLATION_MAX_VELOCITY = maxSpeed; // m/s
        public static final double TRANSLATION_MAX_ACCEL = 3.0; // m/s^2 (tunable)

        /** Motion profile constraints for rotation (radians/sec, radians/sec^2) */
        public static final double ROTATION_MAX_VELOCITY = Math.toRadians(maxTurnSpeed); // rad/s
        public static final double ROTATION_MAX_ACCEL = Math.toRadians(maxTurnSpeed * 2.0); // rad/s^2 (tunable)

        /** Profiled PID controllers for drive-to-pose (constructed from constants) */
        public static final ProfiledPIDController translationProfiledController =
            new ProfiledPIDController(
                translationPidConstants.kP, translationPidConstants.kI, translationPidConstants.kD,
                new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCEL)
            );

        public static final ProfiledPIDController rotationProfiledController =
            new ProfiledPIDController(
                rotationPidConstants.kP, rotationPidConstants.kI, rotationPidConstants.kD,
                new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY, ROTATION_MAX_ACCEL)
            );

    }

    /**
     * Constants for the intake subsystem.
     * Defines motor speeds for game piece intake operations.
     */
    public static final class IntakeSubsystemConstants {
        /** Motor speed in RPM when intaking game pieces (forwards operation). */
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        // public static final AngularVelocity backwardsOnSpeeds = RPM.of(100);
    }

    /**
     * Constants for the shooter subsystem.
     * Includes flywheel speeds and PID constants for velocity control.
     */
    public static final class ShooterSubsystemConstants {
        /** Flywheel motor speed in RPM when shooting game pieces (forwards operation). */
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(3250);
        /** Flywheel motor speed in RPM when shooting game pieces backwards (reverse operation). */
        public static final AngularVelocity backwardsOnSpeeds = RPM.of(-3250);
        /** Manual spin speed in RPM for slow manual control. */
        public static final AngularVelocity manualSpinSpeed = RPM.of(20);
        
        // PID Constants
        /** Proportional gain for shooter velocity PID control. */
        // public static final double SHOOTER_KP = .125;
        // /** Integral gain for shooter velocity PID control. */
        // public static final double SHOOTER_KI = 0.0;
        // /** Derivative gain for shooter velocity PID control. */
        // public static final double SHOOTER_KD = 5.0;

        /** Proportional gain for shooter velocity PID control. */
        public static final double SHOOTER_KP = 0.125;
        /** Integral gain for shooter velocity PID control. */
        public static final double SHOOTER_KI = 0.0;
        /** Derivative gain for shooter velocity PID control. */
        public static final double SHOOTER_KD = 5;

        /** Distance in feet of the middle of the shooting range from the hub */
        public static final double midRange = 2.9
        ;
    }

    /**
     * Constants for the indexer subsystem.
     * Defines motor speeds for game piece indexing and transfer operations.
     */
    public static final class IndexerSubsystemConstants {
        /** Motor speed in RPM when indexing game pieces towards shooter (forwards operation). */
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(1000);
        /** Motor speed in RPM when reversing/ejecting game pieces (backwards operation). */
        public static final AngularVelocity backwardsOnSpeeds = RPM.of(1000);
        /** Manual slow speed in RPM for manual control. */
        public static final AngularVelocity manualSpeed = RPM.of(1000);
    }

    /**
     * Constants for the climber subsystem.
     * Defines position limits for the climbing mechanism.
     */
    public static final class ClimberSubsystemConstants {
        /** Zero/retracted position of the climber (rotations). */
        public static final double ZERO_POSITION = 0;
        /** Climbing position of the climber - between zero and max (rotations). Tune on robot. */
        public static final double CLIMB_POSITION = 10.984;
        /** Maximum extended position of the climber (rotations). Tune on robot. */
        public static final double MAX_POSITION = 18.23;
    }
}