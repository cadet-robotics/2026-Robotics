package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

public class Constants {
    public static final class ControllerConstants {
        // TODO: Configure when controllers arrive
        public static final double deadbandX = 0.15;
        public static final double deadbandY = 0.15;
    }

    public static final class DriveSubsystem {
        // TODO: Configure when robot
        public static final double maxTurnSpeed = 180; // Degrees
        public static final double maxSpeed = 6; // Meters per second

        public static final PIDConstants translationPidConstants = new PIDConstants( 10, 0 , 0 );
        public static final PIDConstants rotationPidConstants = new PIDConstants( 8, 0 , 0 );
    }
}
