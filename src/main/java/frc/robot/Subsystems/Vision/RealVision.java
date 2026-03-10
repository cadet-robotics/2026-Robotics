package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.LimelightHelpers;
import frc.robot.Subsystems.Drive;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.AprilTagFiducial;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

public class RealVision extends SubsystemBase implements Vision {

   private final Drive driveSubsystem;
   private final Limelight back_limelight;
   private final Limelight front_limelight;

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
    * Get the robot's pose on the field using vision.
    * Uses the front limelight as primary and back limelight as backup.
    * Uses botpose_wpiblue for MegaTag2 localization.
    * 
    * @return Optional containing the robot's Pose2d, or empty if no valid pose is
    *         available
    */
   @Override
   public Optional<Pose2d> getRobotPose() {
      // Try front limelight first using LimelightHelpers for MegaTag2 WPIBlue
      double[] front_pose = LimelightHelpers.getBotPose("limelight-front");

      SmartDashboard.putBoolean("Vision/Front Data Null", front_pose == null);

      if (front_pose != null) {
         boolean frontHasTargets = LimelightHelpers.getTV("limelight-front");

         if (front_pose.length >= 6) {
            double x = front_pose[0];
            double y = front_pose[1];
            double rotationDeg = front_pose[5];
            // Only return if pose is non-zero and has tags
            if ((x != 0 || y != 0) && frontHasTargets) {
               Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDeg));
               return Optional.of(pose);
            }
         }
      }

      // Fall back to back limelight
      double[] back_pose = LimelightHelpers.getBotPose("limelight-rear");
      if (back_pose != null) {
         boolean backHasTargets = LimelightHelpers.getTV("limelight-rear");

         if (back_pose.length >= 6) {
            double x = back_pose[0];
            double y = back_pose[1];
            double rotationDeg = back_pose[5];

            if ((x != 0 || y != 0) && backHasTargets) {
               Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDeg));
               return Optional.of(pose);
            }
         }
      }

      return Optional.empty();
   }

   @Override
   public void periodic() {
      Pose2d currentDrivePose = driveSubsystem.getPose();
      double yawDegrees = currentDrivePose.getRotation().getDegrees();
      LimelightHelpers.SetRobotOrientation("limelight-front", yawDegrees, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-rear", yawDegrees, 0, 0, 0, 0, 0);

      if (DriverStation.isDisabled()) {
         LimelightHelpers.PoseEstimate frontEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
         if (frontEstimate != null && frontEstimate.tagCount > 0 &&
               (frontEstimate.pose.getX() != 0 || frontEstimate.pose.getY() != 0)) {

            // Update odometry with vision measurement using the estimate's timestamp
            driveSubsystem.getSwerveDrive().addVisionMeasurement(
                  frontEstimate.pose,
                  frontEstimate.timestampSeconds);
         }
         // Try back limelight as fallback
         LimelightHelpers.PoseEstimate backEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rear");

         if (backEstimate != null && backEstimate.tagCount > 0 &&
               (backEstimate.pose.getX() != 0 || backEstimate.pose.getY() != 0)) {

            // Update odometry with vision measurement using the estimate's timestamp
            driveSubsystem.getSwerveDrive().addVisionMeasurement(
                  backEstimate.pose,
                  backEstimate.timestampSeconds);
         }
      } else {
         LimelightHelpers.PoseEstimate frontEstimate = LimelightHelpers
               .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
         if (frontEstimate != null && frontEstimate.tagCount > 0 &&
               (frontEstimate.pose.getX() != 0 || frontEstimate.pose.getY() != 0)) {

            // Update odometry with vision measurement using the estimate's timestamp
            driveSubsystem.getSwerveDrive().addVisionMeasurement(
                  frontEstimate.pose,
                  frontEstimate.timestampSeconds);
         }
         // Try back limelight as fallback
         LimelightHelpers.PoseEstimate backEstimate = LimelightHelpers
               .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rear");

         if (backEstimate != null && backEstimate.tagCount > 0 &&
               (backEstimate.pose.getX() != 0 || backEstimate.pose.getY() != 0)) {

            // Update odometry with vision measurement using the estimate's timestamp
            driveSubsystem.getSwerveDrive().addVisionMeasurement(
                  backEstimate.pose,
                  backEstimate.timestampSeconds);
         }
      }
   }
}
