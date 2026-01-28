package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Libs.LimelightHelpers;
import frc.robot.Subsystems.Drive;

import java.util.Optional;

public class RealVision implements Vision, Subsystem {

    private final Drive driveSubsystem;
    private NetworkTable table;

    public RealVision(Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    public boolean seesAprilTag() {
        double offset = LimelightHelpers.getTY("");
        // If an offset is present than there must be an apriltag
        return offset != 0;
    }


    /**
     * Changes the pipeline on the limelight to the input id. This is used to switch filters/what ids are accepted.
     *
     * @param id Id of the pipeline
     */
    public void changeFilter( int id ) {
        this.table.getEntry("pipeline").setNumber(id);
    }

    /**
     * A function to get the id of the current april tag.
     *
     * @return Optional containing the ID of the AprilTag, or empty if no tag is visible
     */
    public Optional<Integer> getTagID() {
        if (!seesAprilTag()) {
            return Optional.empty();
        }
        double tagId = this.table.getEntry("tid").getDouble(0.0);
        return tagId > 0 ? Optional.of((int) tagId) : Optional.empty();
    }

    public Optional<Double> getTx() {
        if (!seesAprilTag()) {
            return Optional.empty();
        }
        return Optional.of(this.table.getEntry("tx").getDouble(0.0));
    }

    public Optional<Double> getTy() {
        if (!seesAprilTag()) {
            return Optional.empty();
        }
        return Optional.of(this.table.getEntry("ty").getDouble(0.0));
    }

    public Optional<Double> getTa() {
        if (!seesAprilTag()) {
            return Optional.empty();
        }
        return Optional.of(this.table.getEntry("ta").getDouble(0.0));
    }

    @Override
    public void periodic() {
        Pose2d currentPose = LimelightHelpers.getBotPose2d("");
        this.driveSubsystem.updatePose( currentPose );
        this.table = LimelightHelpers.getLimelightNTTable("limelight");
    }
}
