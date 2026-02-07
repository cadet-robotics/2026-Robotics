package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Libs.LimelightHelpers;
import frc.robot.Subsystems.Drive;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.target.AprilTagFiducial;

import static edu.wpi.first.units.Units.DegreesPerSecond;

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
        double[] front_pose = LimelightHelpers.getBotPose_wpiBlue("limelight-front");
        
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
        double[] back_pose = LimelightHelpers.getBotPose_wpiBlue("limelight-rear");
        
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
        
        // Send robot orientation to limelights for MegaTag2
        // Convert Rotation2d to Rotation3d (Z-axis rotation only)
        Rotation3d robotOrientation = new Rotation3d(0, 0, currentDrivePose.getRotation().getRadians());
        SmartDashboard.putNumber("Vision/Sending Rotation (rad)", robotOrientation.getZ());
        SmartDashboard.putNumber("Vision/Sending Rotation (deg)", Math.toDegrees(robotOrientation.getZ()));
        
        // Create angular velocity (all zeros since we don't have velocity data)
        AngularVelocity3d angularVel = new AngularVelocity3d(
            DegreesPerSecond.of(0),  // roll velocity
            DegreesPerSecond.of(0),  // pitch velocity
            DegreesPerSecond.of(0)   // yaw velocity
        );
        
        // Update both limelights with robot orientation
        front_limelight.getSettings()
            .withRobotOrientation(new Orientation3d(robotOrientation, angularVel))
            .save();
            
        back_limelight.getSettings()
            .withRobotOrientation(new Orientation3d(robotOrientation, angularVel))
            .save();
        
        Optional<Pose2d> robotPose = getRobotPose();
        
        if (robotPose.isPresent()) {
            Pose2d pose = robotPose.get();
            
            this.driveSubsystem.updatePose(pose);
            
            // Publish pose to SmartDashboard as a single pose element
            SmartDashboard.putNumberArray("Vision/Robot Pose", new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians()
            });
            SmartDashboard.putBoolean("Vision/Has Valid Pose", true);
        } else {
            SmartDashboard.putBoolean("Vision/Has Valid Pose", false);
        }
    }
}
