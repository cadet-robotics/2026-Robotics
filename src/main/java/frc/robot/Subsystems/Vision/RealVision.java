package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;

public class RealVision extends SubsystemBase implements Vision {

    private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    private final StructPublisher<Pose2d> frontPosePublisher = visionTable.getStructTopic("frontPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> backPosePublisher = visionTable.getStructTopic("backPose", Pose2d.struct).publish();
    private final BooleanPublisher frontSeesTag = visionTable.getBooleanTopic("frontSeesTag").publish();
    private final BooleanPublisher backSeesTag = visionTable.getBooleanTopic("backSeesTag").publish();
    private final IntegerPublisher frontTagCount = visionTable.getIntegerTopic("frontTagCount").publish();
    private final IntegerPublisher backTagCount = visionTable.getIntegerTopic("backTagCount").publish();
    
    private final Drive driveSubsystem;
    private final Limelight back_limelight;
    private final Limelight front_limelight;

    private final LimelightPoseEstimator frontPoseEstimatorM1;
    private final LimelightPoseEstimator frontPoseEstimatorM2;
    private final LimelightPoseEstimator backPoseEstimatorM1;
    private final LimelightPoseEstimator backPoseEstimatorM2;

    private boolean seeded = false;

    public RealVision(Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.front_limelight = new Limelight("limelight-front");
        this.back_limelight = new Limelight("limelight-rear");
        this.frontPoseEstimatorM1 = new LimelightPoseEstimator(front_limelight, LimelightPoseEstimator.EstimationMode.MEGATAG1);
        this.frontPoseEstimatorM2 = new LimelightPoseEstimator(front_limelight, LimelightPoseEstimator.EstimationMode.MEGATAG2);
        this.backPoseEstimatorM1 = new LimelightPoseEstimator(back_limelight, LimelightPoseEstimator.EstimationMode.MEGATAG1);
        this.backPoseEstimatorM2 = new LimelightPoseEstimator(back_limelight, LimelightPoseEstimator.EstimationMode.MEGATAG2);
    }

    public boolean seesAprilTag() {
        boolean frontSees = frontSeesAprilTag();
        boolean backSees = backSeesAprilTag();
        frontSeesTag.set(frontSees);
        frontTagCount.set(frontSees ? front_limelight.getLatestResults().get().targets_Fiducials.length : 0);
        backSeesTag.set(backSees);
        backTagCount.set(backSees ? back_limelight.getLatestResults().get().targets_Fiducials.length : 0);
        return frontSees || backSees;
    }

    public boolean frontSeesAprilTag() {
        return front_limelight.getLatestResults().isPresent()
            && front_limelight.getLatestResults().get().targets_Fiducials != null
            && front_limelight.getLatestResults().get().targets_Fiducials.length > 0;
    }

    public boolean backSeesAprilTag() {
        return back_limelight.getLatestResults().isPresent()
            && back_limelight.getLatestResults().get().targets_Fiducials != null
            && back_limelight.getLatestResults().get().targets_Fiducials.length > 0;
    }

    /**
     * Get the robot's pose on the field using vision.
     * Uses the front limelight as primary and back limelight as backup.
     * Uses botpose_wpiblue for MegaTag2 localization.
     * @return Optional containing the robot's Pose2d, or empty if no valid pose is available
     */
    @Override
    public Optional<Pose2d> getRobotPose() {
        if ( frontPoseEstimatorM2.getPoseEstimate().isPresent() ) {
            Pose2d frontPose = frontPoseEstimatorM2.getPoseEstimate().get().pose.toPose2d();
            frontPosePublisher.set(frontPose);
            return Optional.of(frontPose);
        }
        if ( backPoseEstimatorM2.getPoseEstimate().isPresent() ) {
            Pose2d backPose = backPoseEstimatorM2.getPoseEstimate().get().pose.toPose2d();
            backPosePublisher.set(backPose);
            return Optional.of(backPose);
        }
        return Optional.empty();
    }

    public void seed() {
        Optional<PoseEstimate> limelight1seed = frontPoseEstimatorM1.getPoseEstimate();
        Optional<PoseEstimate> limelight2seed = frontPoseEstimatorM2.getPoseEstimate();

        if (limelight1seed.isPresent()) {
            driveSubsystem.updatePose(limelight1seed.get().pose.toPose2d(), limelight1seed.get().timestampSeconds);
            seeded = true;
        } else if (limelight2seed.isPresent()) {
            driveSubsystem.updatePose(limelight2seed.get().pose.toPose2d(), limelight2seed.get().timestampSeconds);
            seeded = true;
        }
    }

    public void disbaledPeriodic() {

        Orientation3d robotOrientation = new Orientation3d( new Rotation3d(driveSubsystem.getSwerveDrive().getOdometryHeading()), new AngularVelocity3d(RPM.of(0), RPM.of(0), RPM.of(0)));
        front_limelight.getSettings().withRobotOrientation(robotOrientation);
        back_limelight.getSettings().withRobotOrientation(robotOrientation);
    }

    @Override
    public void periodic() {
        seesAprilTag();

        Orientation3d robotOrientation = new Orientation3d( new Rotation3d(driveSubsystem.getSwerveDrive().getOdometryHeading()), new AngularVelocity3d(RPM.of(0), RPM.of(0), RPM.of(0)));
        front_limelight.getSettings().withRobotOrientation(robotOrientation);
        back_limelight.getSettings().withRobotOrientation(robotOrientation);

        if (!seeded) {
            seed();
        } else {
            Optional<Pose2d> shrodingersPose = getRobotPose();
            if (shrodingersPose.isPresent()) {
                Pose2d pose = shrodingersPose.get();
                driveSubsystem.updatePose(pose, frontSeesAprilTag() 
                    ? frontPoseEstimatorM2.getPoseEstimate().get().timestampSeconds
                    : backPoseEstimatorM2.getPoseEstimate().get().timestampSeconds
                );
            }
        }
    }
}
