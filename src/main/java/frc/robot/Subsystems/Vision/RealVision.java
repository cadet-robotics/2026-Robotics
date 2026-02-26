package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Libs.LimelightHelpers;
import frc.robot.Subsystems.Drive;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.AprilTagFiducial;

import java.util.Optional;

public class RealVision implements Vision, Subsystem {

    private final Drive driveSubsystem;
    private final Limelight back_limelight;
    private final Limelight front_limelight;
    private int periodicCounter = 0;

    public RealVision(Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.front_limelight = new Limelight("limelight-front");
        this.back_limelight = new Limelight("limelight-rear");
        
        // Add debug message to confirm subsystem is created
        SmartDashboard.putString("Vision/Status", "Vision Subsystem Initialized");
        SmartDashboard.putBoolean("Vision/Subsystem Active", true);
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

    /**
     * Get the robot's pose on the field using vision.
     * Uses the front limelight as primary and back limelight as backup.
     * Uses botpose_wpiblue for MegaTag2 localization.
     * @return Optional containing the robot's Pose2d, or empty if no valid pose is available
     */
    @Override
    public Optional<Pose2d> getRobotPose() {
        // Try front limelight first using LimelightHelpers for MegaTag2 WPIBlue
        double[] front_pose = LimelightHelpers.getBotPose("limelight-front");
        
        SmartDashboard.putBoolean("Vision/Front Data Null", front_pose == null);
        
        if (front_pose != null) {
            SmartDashboard.putBoolean("Vision/Front wpiblue Null", false);
            SmartDashboard.putBoolean("Vision/Front wpiblue Length >= 6", front_pose.length >= 6);
            
            boolean frontHasTargets = LimelightHelpers.getTV("limelight-front");
            SmartDashboard.putBoolean("Vision/Front Tag Count > 0", frontHasTargets);
            
            if (front_pose.length >= 6) {
                double x = front_pose[0];
                double y = front_pose[1];
                double rotationDeg = front_pose[5];
                
                SmartDashboard.putBoolean("Vision/Front Pose Is Zero", (x == 0 && y == 0));
                SmartDashboard.putBoolean("Vision/Front Has Tags", frontHasTargets);
                
                // Only return if pose is non-zero and has tags
                if ((x != 0 || y != 0) && frontHasTargets) {
                    Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDeg));
                    SmartDashboard.putString("Vision/Active Limelight", "Front");
                    SmartDashboard.putNumberArray("Vision/Front Pose", new double[] {
                        pose.getX(),
                        pose.getY(),
                        pose.getRotation().getRadians()
                    });
                    return Optional.of(pose);
                }
            }
        }
        
        // Fall back to back limelight
        double[] back_pose = LimelightHelpers.getBotPose("limelight-rear");
        
        SmartDashboard.putBoolean("Vision/Back Data Null", back_pose == null);
        
        if (back_pose != null) {
            SmartDashboard.putBoolean("Vision/Back wpiblue Null", false);
            SmartDashboard.putBoolean("Vision/Back wpiblue Length >= 6", back_pose.length >= 6);
            
            boolean backHasTargets = LimelightHelpers.getTV("limelight-rear");
            SmartDashboard.putBoolean("Vision/Back Tag Count > 0", backHasTargets);
            
            if (back_pose.length >= 6) {
                double x = back_pose[0];
                double y = back_pose[1];
                double rotationDeg = back_pose[5];
                
                SmartDashboard.putBoolean("Vision/Back Pose Is Zero", (x == 0 && y == 0));
                SmartDashboard.putBoolean("Vision/Back Has Tags", backHasTargets);
                
                if ((x != 0 || y != 0) && backHasTargets) {
                    Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDeg));
                    SmartDashboard.putString("Vision/Active Limelight", "Back");
                    SmartDashboard.putNumberArray("Vision/Back Pose", new double[] {
                        pose.getX(),
                        pose.getY(),
                        pose.getRotation().getRadians()
                    });
                    return Optional.of(pose);
                }
            }
        }
        
        SmartDashboard.putString("Vision/Active Limelight", "None");
        return Optional.empty();
    }

    @Override
    public void periodic() {

        periodicCounter++;
        SmartDashboard.putNumber("Vision/Periodic Counter", periodicCounter);
        
        // Log current drive pose
        Pose2d currentDrivePose = driveSubsystem.getPose();
        SmartDashboard.putNumber("Vision/Drive Pose X", currentDrivePose.getX());
        SmartDashboard.putNumber("Vision/Drive Pose Y", currentDrivePose.getY());
        SmartDashboard.putNumber("Vision/Drive Pose Rotation", currentDrivePose.getRotation().getDegrees());
        
        // Update robot orientation in NetworkTables for both limelights BEFORE fetching pose estimates
        // This ensures MegaTag2 has the latest odometry data
        double yawDegrees = currentDrivePose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation("limelight-front", yawDegrees, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-rear", yawDegrees, 0, 0, 0, 0, 0);
        
        SmartDashboard.putNumber("Vision/Sending Rotation (deg)", yawDegrees);
        
        // Try to get pose estimate from front limelight using MegaTag2
        LimelightHelpers.PoseEstimate frontEstimate = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        
        if (frontEstimate != null && frontEstimate.tagCount > 0 && 
            (frontEstimate.pose.getX() != 0 || frontEstimate.pose.getY() != 0)) {
            
            // Update odometry with vision measurement using the estimate's timestamp
            driveSubsystem.getSwerveDrive().addVisionMeasurement(
                frontEstimate.pose, 
                frontEstimate.timestampSeconds
            );
            
            // Publish pose to SmartDashboard
            SmartDashboard.putString("Vision/Active Limelight", "Front");
            SmartDashboard.putNumberArray("Vision/Robot Pose", new double[] {
                frontEstimate.pose.getX(),
                frontEstimate.pose.getY(),
                frontEstimate.pose.getRotation().getRadians()
            });
            SmartDashboard.putNumber("Vision/Timestamp", frontEstimate.timestampSeconds);
            SmartDashboard.putNumber("Vision/Tag Count", frontEstimate.tagCount);
            SmartDashboard.putBoolean("Vision/Has Valid Pose", true);
            
        } else {
            // Try back limelight as fallback
            LimelightHelpers.PoseEstimate backEstimate = 
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rear");
            
            if (backEstimate != null && backEstimate.tagCount > 0 && 
                (backEstimate.pose.getX() != 0 || backEstimate.pose.getY() != 0)) {
                
                // Update odometry with vision measurement using the estimate's timestamp
                driveSubsystem.getSwerveDrive().addVisionMeasurement(
                    backEstimate.pose, 
                    backEstimate.timestampSeconds
                );
                
                // Publish pose to SmartDashboard
                SmartDashboard.putString("Vision/Active Limelight", "Back");
                SmartDashboard.putNumberArray("Vision/Robot Pose", new double[] {
                    backEstimate.pose.getX(),
                    backEstimate.pose.getY(),
                    backEstimate.pose.getRotation().getRadians()
                });
                SmartDashboard.putNumber("Vision/Timestamp", backEstimate.timestampSeconds);
                SmartDashboard.putNumber("Vision/Tag Count", backEstimate.tagCount);
                SmartDashboard.putBoolean("Vision/Has Valid Pose", true);
                
            } else {
                SmartDashboard.putString("Vision/Active Limelight", "None");
                SmartDashboard.putBoolean("Vision/Has Valid Pose", false);
            }
        }
    }
}
