package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Drive;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.AprilTagFiducial;

import java.util.Optional;

public class RealVision implements Vision, Subsystem {

    private final Drive driveSubsystem;
    private final Limelight back_limelight;
    private final Limelight front_limelight;

    public RealVision(Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.front_limelight = new Limelight("front_limelight");
        this.back_limelight = new Limelight("back_limelight");
    }

    public boolean seesAprilTag() {
        Optional<LimelightResults> front_results = front_limelight.getLatestResults();
        if (front_results.isPresent()) {
            LimelightResults data = front_results.get();
            return data.targets_Fiducials != null && data.targets_Fiducials.length > 0;
        }
        Optional<LimelightResults> back_results = back_limelight.getLatestResults();
        if (back_results.isPresent()) {
            LimelightResults data = back_results.get();
            return data.targets_Fiducials != null && data.targets_Fiducials.length > 0;
        }
        return false;
    }

    /**
     * Changes the pipeline on the limelight to the input id. This is used to switch filters/what ids are accepted.
     *
     * @param id Id of the pipeline
     */
    public void changeFilter(int id) {
        front_limelight.getSettings()
                .withPipelineIndex(id)
                .save();
    }

    /**
     * A function to get the id of the current april tag.
     *
     * @return Optional containing the ID of the AprilTag, or empty if no tag is visible
     */
    public Optional<Integer> getTagID() {
        Optional<LimelightResults> results = front_limelight.getLatestResults();
        if (results.isPresent() && seesAprilTag()) {
            AprilTagFiducial[] tags = results.get().targets_Fiducials;
            if (tags.length > 0) {
                return Optional.of((int) tags[0].fiducialID);
            }
        }
        return Optional.empty();
    }

    public Optional<Double> getTx() {
        Optional<LimelightResults> results = front_limelight.getLatestResults();
        if (results.isPresent() && seesAprilTag()) {
            AprilTagFiducial[] tags = results.get().targets_Fiducials;
            if (tags.length > 0) {
                return Optional.of(tags[0].tx);
            }
        }
        return Optional.empty();
    }

    public Optional<Double> getTy() {
        Optional<LimelightResults> results = front_limelight.getLatestResults();
        if (results.isPresent() && seesAprilTag()) {
            AprilTagFiducial[] tags = results.get().targets_Fiducials;
            if (tags.length > 0) {
                return Optional.of(tags[0].ty);
            }
        }
        return Optional.empty();
    }

    public Optional<Double> getTa() {
        Optional<LimelightResults> results = front_limelight.getLatestResults();
        if (results.isPresent() && seesAprilTag()) {
            AprilTagFiducial[] tags = results.get().targets_Fiducials;
            if (tags.length > 0) {
                return Optional.of(tags[0].ta);
            }
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        Optional<LimelightResults> front_results = front_limelight.getLatestResults();
        if (front_results.isPresent()) {
            Pose2d currentPose = front_results.get().getBotPose2d();
            this.driveSubsystem.updatePose(currentPose);
        } else {
            Optional<LimelightResults> back_results = front_limelight.getLatestResults();
            if (back_results.isPresent()) {
                Pose2d currentPose = back_results.get().getBotPose2d();
                this.driveSubsystem.updatePose(currentPose);
            }
        }
    }
}
