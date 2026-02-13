package frc.robot.Constants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RPM;

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
        // TODO: Configure when controllers arrive
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
        public static final Translation2d BLUE_HUB_POSITION = new Translation2d(182.11, 158.84);
        /** Red alliance hub/speaker position (x, y) in inches. */
        public static final Translation2d RED_HUB_POSITION = new Translation2d(469.11, 158.84);
    }

    /**
     * Constants for the swerve drive subsystem.
     * Includes maximum speeds and PID constants for translation and rotation control.
     */
    public static final class DriveSubsystemConstants {
        // TODO: Configure when robot
        /** Maximum rotational speed of the robot in degrees per second. */
        public static final double maxTurnSpeed = 90; // Degrees
        /** Maximum translational speed of the robot in meters per second. */
        public static final double maxSpeed = 6; // Meters per second

        /** PID constants for translational (x, y) movement control. */
        public static final PIDConstants translationPidConstants = new PIDConstants(0.00000104, 0, 0);
        /** PID constants for rotational (theta) movement control. */
        public static final PIDConstants rotationPidConstants = new PIDConstants(0.000000005, 0, 0);
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
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        /** Flywheel motor speed in RPM when shooting game pieces backwards (reverse operation). */
        public static final AngularVelocity backwardsOnSpeeds = RPM.of(-100);
        /** Manual spin speed in RPM for slow manual control. */
        public static final AngularVelocity manualSpinSpeed = RPM.of(20);
        
        // PID Constants
        /** Proportional gain for shooter velocity PID control. */
        public static final double SHOOTER_KP = 50;
        /** Integral gain for shooter velocity PID control. */
        public static final double SHOOTER_KI = 0.0;
        /** Derivative gain for shooter velocity PID control. */
        public static final double SHOOTER_KD = 0.0;
    }

    /**
     * Constants for the indexer subsystem.
     * Defines motor speeds for game piece indexing and transfer operations.
     */
    public static final class IndexerSubsystemConstants {
        /** Motor speed in RPM when indexing game pieces towards shooter (forwards operation). */
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        /** Motor speed in RPM when reversing/ejecting game pieces (backwards operation). */
        public static final AngularVelocity backwardsOnSpeeds = RPM.of(100);
        /** Manual slow speed in RPM for manual control. */
        public static final AngularVelocity manualSpeed = RPM.of(1000);
    }
}
