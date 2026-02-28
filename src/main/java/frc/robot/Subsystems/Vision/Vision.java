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
     * Changes the pipeline on the vision system to the input id. This is used to switch filters/what ids are accepted.
     *
     * @param id Id of the pipeline
     */
    public void changeFilter(int id);

    /**
     * A function to get the id of the current april tag.
     *
     * @return Optional containing the ID of the AprilTag, or empty if no tag is visible
     */
    public Optional<Integer> getTagID();

    /**
     * Get the horizontal offset to the target (yaw)
     * @return Optional containing the tx value in degrees, or empty if no target is visible
     */
    public Optional<Double> getTx();

    /**
     * Get the vertical offset to the target (pitch)
     * @return Optional containing the ty value in degrees, or empty if no target is visible
     */
    public Optional<Double> getTy();

    /**
     * Get the target area
     * @return Optional containing the ta value (percent of image), or empty if no target is visible
     */
    public Optional<Double> getTa();

    /**
     * Get the robot's pose on the field using vision.
     * Uses the front limelight as primary and back limelight as backup.
     * @return Optional containing the robot's Pose2d, or empty if no valid pose is available
     */
    public Optional<Pose2d> getRobotPose();

}
