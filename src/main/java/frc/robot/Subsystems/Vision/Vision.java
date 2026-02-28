package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;

public interface Vision {

    /**
     * Check if the vision system currently sees an AprilTag
     * @return true if an AprilTag is visible
     */
    public boolean seesAprilTag();

    /**
     * Get the robot's pose on the field using vision.
     * Uses the front limelight as primary and back limelight as backup.
     * @return Optional containing the robot's Pose2d, or empty if no valid pose is available
     */
    public Optional<Pose2d> getRobotPose();

    public default void disabledPeriodic() {
        // Default implementation does nothing, but can be overridden by implementations that need to update in disabled mode (e.g. to keep limelights seeded with robot pose)
    }

}
