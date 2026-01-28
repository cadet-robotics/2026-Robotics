package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RPM;

public class Constants {
    public static final class ControllerConstants {
        // TODO: Configure when controllers arrive
        public static final double deadbandX = 0.15;
        public static final double deadbandY = 0.15;
    }

    public static final class DriveSubsystemConstants {
        // TODO: Configure when robot
        public static final double maxTurnSpeed = 90; // Degrees
        public static final double maxSpeed = 6; // Meters per second

        public static final PIDConstants translationPidConstants = new PIDConstants(10, 0, 0);
        public static final PIDConstants rotationPidConstants = new PIDConstants(8, 0, 0);
    }

    public static final class IntakeSubsystemConstants {
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        // public static final AngularVelocity backwardsOnSpeeds = RPM.of(100);
    }

    public static final class ShooterSubsystemConstants {
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        // public static final AngularVelocity backwardsOnSpeeds = RPM.of(100);
        
        // PID Constants
        public static final double SHOOTER_KP = 50.0;
        public static final double SHOOTER_KI = 0.0;
        public static final double SHOOTER_KD = 0.0;
    }

    public static final class IndexerSubsystemConstants {
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        public static final AngularVelocity backwardsOnSpeeds = RPM.of(100);
    }
}
