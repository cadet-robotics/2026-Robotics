package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Libs.LimelightHelpers;

public class Vision implements Subsystem {

    private final Drive driveSubsystem;
    private NetworkTable table;

    public Vision(Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    public boolean seesAprilTag() {
        double offset = LimelightHelpers.getTY("");
        // If an offset is present than there must be an apriltag
        return offset != 0;
    }

    /**
     * Changes the pipeline on the limelight to the input id. This is used to switch filters/what ids are excepted.
     *
     * @param id Id of the pipeline
     */
    public void changeFilter( int id ) {
        LimelightHelpers.getLimelightNTTable("limelight").getEntry("pipeline").setNumber(id);
    }

    /**
     * A function to get the id of the current april tag.
     *
     * @return int Id of the AprilTag
     */
    public double getTagID() {
        return this.table.getEntry("tid").getDouble(0.0);
    }

    public double getTx() {
        return this.table.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return this.table.getEntry("ty").getDouble(0.0);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = LimelightHelpers.getBotPose2d("");
        this.driveSubsystem.updatePose( currentPose );
        this.table = LimelightHelpers.getLimelightNTTable("limelight");
    }
}
