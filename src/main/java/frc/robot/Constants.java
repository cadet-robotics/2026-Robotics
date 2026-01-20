package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Preferences;

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
        
        // PID Constants - loaded from persistent storage or defaults
        private static final String SHOOTER_KP_KEY = "Shooter/PID/Kp";
        private static final String SHOOTER_KI_KEY = "Shooter/PID/Ki";
        private static final String SHOOTER_KD_KEY = "Shooter/PID/Kd";
        
        // Default PID values (fallback if not autotuned)
        private static final double DEFAULT_KP = 50.0;
        private static final double DEFAULT_KI = 0.0;
        private static final double DEFAULT_KD = 0.0;
        
        /**
         * Get shooter Kp value from persistent storage or default
         */
        public static double getShooterKp() {
            return Preferences.getDouble(SHOOTER_KP_KEY, DEFAULT_KP);
        }
        
        /**
         * Get shooter Ki value from persistent storage or default
         */
        public static double getShooterKi() {
            return Preferences.getDouble(SHOOTER_KI_KEY, DEFAULT_KI);
        }
        
        /**
         * Get shooter Kd value from persistent storage or default
         */
        public static double getShooterKd() {
            return Preferences.getDouble(SHOOTER_KD_KEY, DEFAULT_KD);
        }
        
        /**
         * Save shooter PID values to persistent storage
         */
        public static void saveShooterPID(double kp, double ki, double kd) {
            Preferences.setDouble(SHOOTER_KP_KEY, kp);
            Preferences.setDouble(SHOOTER_KI_KEY, ki);
            Preferences.setDouble(SHOOTER_KD_KEY, kd);
            System.out.println("Saved Shooter PID: Kp=" + kp + " Ki=" + ki + " Kd=" + kd);
        }
        
        /**
         * Reset shooter PID to defaults
         */
        public static void resetShooterPIDToDefaults() {
            saveShooterPID(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
        }
    }

    public static final class IndexerSubsystemConstants {
        public static final AngularVelocity forwardsOnSpeeds = RPM.of(100);
        public static final AngularVelocity backwardsOnSpeeds = RPM.of(100);
    }
}
