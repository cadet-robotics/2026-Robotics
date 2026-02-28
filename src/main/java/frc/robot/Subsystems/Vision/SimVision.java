package frc.robot.Subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive;

import java.util.List;
import java.util.Optional;

/**
 * PhotonVision-based simulation vision system for AprilTag detection and pose estimation.
 * This replaces Limelight in simulation with PhotonVision's simulation capabilities.
 */
public class SimVision extends SubsystemBase implements Vision {

    private final Drive driveSubsystem;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d robotToCamera;

    public SimVision(Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        // Load the 2025 Reefscape field AprilTag layout
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Create PhotonCamera instance
        this.camera = new PhotonCamera("SimCamera");

        // Create robot-to-camera transform
        // Camera mounted 0.25m forward, 0.5m up, tilted down 20 degrees
        this.robotToCamera = new Transform3d(
            new Translation3d(0.25, 0.0, 0.5),   // camera position on robot (forward, left, up)
            new Rotation3d(0, Math.toRadians(-20), 0)  // pitch down 20 degrees
        );
        
        // Create pose estimator - PhotonPoseEstimator constructor takes (fieldLayout, strategy, transform)
        this.poseEstimator = new PhotonPoseEstimator(
            this.fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  // Best strategy for multiple tags
            this.robotToCamera
        );
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Create vision system simulation
        this.visionSim = new VisionSystemSim("main");
        this.visionSim.addAprilTags(this.fieldLayout);

        // Configure simulated camera properties
        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(960, 720, Rotation2d.kZero);  // Resolution and calibration
        cameraProps.setCalibError(0.25, 0.08);  // Average error in pixels, std dev
        cameraProps.setFPS(90);  // 90 FPS camera
        cameraProps.setAvgLatencyMs(20);  // 20ms average latency
        cameraProps.setLatencyStdDevMs(5);  // 5ms latency std dev
        
        // Create camera simulation
        this.cameraSim = new PhotonCameraSim(this.camera, cameraProps);
        this.cameraSim.enableProcessedStream(true);
        
        // Add camera to vision system
        this.visionSim.addCamera(this.cameraSim, this.robotToCamera);
        
        // Enable drawing wireframe on field
        this.cameraSim.enableDrawWireframe(true);
    }

    /**
     * Get the latest pipeline result from the camera
     * @return Optional containing the latest result, or empty if no results available
     */
    private Optional<PhotonPipelineResult> getLatestResult() {
        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(results.get(results.size() - 1));  // Get most recent result
    }

    @Override
    public boolean seesAprilTag() {
        return getLatestResult().map(PhotonPipelineResult::hasTargets).orElse(false);
    }

    @Override
    public Optional<Pose2d> getRobotPose() {
        // Get the latest result from the camera
        Optional<PhotonPipelineResult> result = getLatestResult();
        if (result.isPresent() && result.get().hasTargets()) {
            // Use the pose estimator to get the robot's pose
            return this.poseEstimator.update(result.get())
                .map(estimatedPose -> estimatedPose.estimatedPose.toPose2d());
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        // Update robot pose in the vision simulation based on drive subsystem's odometry
        Pose2d currentOdometryPose = this.driveSubsystem.getPose();
        this.visionSim.update(currentOdometryPose);
        
        // Get all unread results from the camera
        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        
        // Process the most recent result if available
        if (!results.isEmpty()) {
            PhotonPipelineResult latestResult = results.get(results.size() - 1);
            if (latestResult.hasTargets()) {
                this.poseEstimator.update(latestResult).ifPresent(estimatedPose -> {
                    // Update drive subsystem with vision-based pose estimate
                    Pose2d visionPose = estimatedPose.estimatedPose.toPose2d();
                    this.driveSubsystem.updatePose(visionPose, estimatedPose.timestampSeconds);
                });
            }
        }
    }
    
    /**
     * Get the vision system simulator (useful for debugging and visualization)
     * @return The VisionSystemSim instance
     */
    public VisionSystemSim getVisionSim() {
        return this.visionSim;
    }
    
    /**
     * Get the camera simulator (useful for advanced configuration)
     * @return The PhotonCameraSim instance
     */
    public PhotonCameraSim getCameraSim() {
        return this.cameraSim;
    }
}
