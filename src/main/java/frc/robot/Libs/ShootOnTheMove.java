// blog.eeshwark.com/blog/shooting-on-fly
package frc.robot.Libs;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.RobotConstants.FieldConstants;

public class ShootOnTheMove {

    private static final NetworkTable STOMTable = NetworkTableInstance.getDefault().getTable("STOM");
    private static final BooleanPublisher RPM_Exisits = STOMTable.getBooleanTopic("RPM_Exists").publish();
    private static final BooleanPublisher Heading_Exists = STOMTable.getBooleanTopic("Heading_Exists").publish();
    private static final DoublePublisher RPM_PUBLISHER = STOMTable.getDoubleTopic("RPM").publish();
    private static final StructPublisher<Rotation2d> HEADING_PUBLISHER = STOMTable.getStructTopic("HEADING", Rotation2d.struct).publish();

    public static Optional<AngularVelocity> shooterRPM = Optional.empty();
    public static Optional<Rotation2d> heading = Optional.empty();

    /**
     * Publishes the calculated shooter RPM and lead heading to NetworkTables for use in the Drive subsystem. Also publishes whether or not these values exist, so that the Drive subsystem can choose whether or not to use them.
     */
    public static void publish() {
        RPM_Exisits.set(shooterRPM.isPresent());
        Heading_Exists.set(heading.isPresent());
        shooterRPM.ifPresent(rpm -> RPM_PUBLISHER.set(shooterRPM.get().magnitude()));
        heading.ifPresent(h -> HEADING_PUBLISHER.set(h));
    }

    public static double calculateShooterRPM(double distanceToTarget) {
        // The following is a placeholder for sim, these values are entirely made up.
        // If this works on the real robot it is nothing short of a miracle.
        return distanceToTarget * 3000 / 2.5;
    }

    public static double calculateIdealHorizontalSpeed(double distance) {
        // The following is a placeholder for sim, these values are entirely made up.
        // If this works on the real robot it is nothing short of a miracle.
        return distance * 0.8;
    }

    public static void calculateLeadHeading(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        Pose2d hubPose = FieldConstants.hub.get();

        double targetX = hubPose.getX() - robotPose.getX();
        double targetY = hubPose.getY() - robotPose.getY();
        Translation2d targetPosition = new Translation2d( targetX, targetY );
    
        double distance = targetPosition.getNorm();
        double idealSpeeds = calculateIdealHorizontalSpeed(distance);
        
        double releaseAngle = 80; // The angle of incline of a shot. Must actually be found.
        double idealSpeeds_Horizontal = idealSpeeds * Math.cos(Math.toRadians(releaseAngle));

        Translation2d targetVector = targetPosition.div(distance).times(idealSpeeds_Horizontal);
        
        Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        Translation2d shotVector = targetVector.div(distance).minus(robotVelocity);

        double robotHeading = shotVector.getAngle().getDegrees();
        // heading = Optional.of(Rotation2d.fromDegrees(robotHeading));

        double requiredSpeed = shotVector.getNorm();
        double shooterRPM_ = calculateShooterRPM(requiredSpeed);
        // shooterRPM = Optional.of(RPM.of(shooterRPM_));
    }
}
