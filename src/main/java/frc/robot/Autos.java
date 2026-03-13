package frc.robot; 

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Drive;
import swervelib.SwerveInputStream;

/**
 * Class for managing autonomous routines.
 * Provides auto selection through a SendableChooser and individual auto commands.
 */
public class Autos {
    private SendableChooser<Command> autoChooser;
    private HashMap<String, Supplier<Command>> namedCommands = new HashMap<>();
    // Track the last selection from the SendableChooser so we only recompute when it changes

    private Drive drive_subsystem;
    /**
     * Constructs the Autos object and initializes the auto chooser.
     * 
     * @param robotContainer the robot container instance
     * @param driveSubsystem the drive subsystem instance
     */
    public Autos( RobotContainer robotContainer, Drive driveSubsystem ) {
        // AutoBuilder might not be configured if PathPlanner config has errors
        this.drive_subsystem = driveSubsystem;
        try {
            autoChooser = AutoBuilder.buildAutoChooser();
        } catch (RuntimeException e) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "Could not build auto chooser - PathPlanner may not be configured: " + e.getMessage(), 
                false
            );
            autoChooser = null;
        }
    }

    /**
     * A Supplier-like functional interface that can throw checked exceptions.
     */
    @FunctionalInterface
    private interface ThrowingSupplier<T> {
        T get() throws Exception;
    }

    /**
     * Wrap a supplier that may throw checked exceptions and return a safe command.
     * Any exception thrown will be reported to the DriverStation and a no-op command
     * will be returned so the robot remains stable.
     */
    public Command autoWrapper(ThrowingSupplier<Command> command) {
        try {
            return command.get();
        } catch (Exception e) {
            DriverStation.reportError("Auto creation failed: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    /**
     * Gets the staring pose from a {@link PathPlannerPath} using the current alliance.
     * 
     * @param path the path to get the starting pose from
     * @return the starting pose
     */
    public Pose2d getStartingPoseFromPath(PathPlannerPath path) {
        if (Dashboard.getAlliance() == Alliance.Blue ) {
            return path.getStartingHolonomicPose().get();
        } else {
            return path.flipPath().getStartingHolonomicPose().get();
        }
    }

    /**
     * Gets the ending pose from a {@link PathPlannerPath} using the current alliance.
     * 
     * @param path the path to get the ending pose from
     * @return the ending pose
     */
    public Pose2d getEndPoseFromPath(PathPlannerPath path) {
        if (Dashboard.getAlliance() == Alliance.Blue ) {
            List<Pose2d> pathPoints = path.getPathPoses();
            Rotation2d pointRotation = path.getGoalEndState().rotation();
            return new Pose2d(pathPoints.get(pathPoints.size() - 1).getTranslation(), pointRotation);
        } else {
            List<Pose2d> pathPoints = path.flipPath().getPathPoses();
            Rotation2d pointRotation = path.flipPath().getGoalEndState().rotation();
            return new Pose2d(pathPoints.get(pathPoints.size() - 1).getTranslation(), pointRotation);
        }
    }

    /**
     * Wraps a follow {@link PathPlannerPath} in a pathplanner drive to pose command for the staring and ending poses.
     *
     * @param path the path to follow
     * @return the seqential command group.
     */
    public Command pathPlannerDtpPath(PathPlannerPath path) {
        return Commands.sequence(
            drive_subsystem.driveToTargetPose(getStartingPoseFromPath(path), 0),
            AutoBuilder.followPath(path),
            drive_subsystem.driveToTargetPose(getEndPoseFromPath(path), 0)
        );
    }

    /**
     * Adds a named command the the auto's command hashmap.
     * 
     * @param name the name of the command
     * @param command the command to add
     */
    public void addCommand( String name, Supplier<Command> command) {
        namedCommands.put(name, command);
    }

    /**
     * Inits all of the autos to the {@link SenableChooser}.
     */
    public void addAutos() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));
        autoChooser.addOption("RightShootClimb", autoWrapper(this::rightTrifecta).withName("RightShootClimb"));
        autoChooser.addOption("RightRefilTrifecta", autoWrapper(this::rightRefilTrifecta).withName("RightRefilTrifecta"));
        autoChooser.addOption("InvertedRightRefilTrifecta", autoWrapper(this::invertedRightRefilTrifecta).withName("InvertedRightRefilTrifecta"));
        autoChooser.addOption("LeftShootClimb", autoWrapper(this::leftTrifecta).withName("LeftShootClimb"));
        autoChooser.addOption("RightAllOfTheMarbles", autoWrapper(this::rightCollectShootClimb).withName("RightAllOfTheMarbles"));
        autoChooser.addOption("RightShoot", autoWrapper(this::rightShoot).withName("RightShoot"));
        autoChooser.addOption("MiddleShoot", autoWrapper(this::middleShoot).withName("MiddleShoot"));
        autoChooser.addOption("MiddleShootRightClimb", autoWrapper(this::middleShootRightClimb).withName("MiddleShootRightClimb"));
        autoChooser.addOption("MiddleShootLeftClimb", autoWrapper(this::middleShootLeftClimb).withName("MiddleShootLeftClimb"));
        autoChooser.addOption("MiddleRightClimb", autoWrapper(this::middleRightClimb).withName("MiddleRightClimb"));
        autoChooser.addOption("MiddleLeftClimb", autoWrapper(this::middleLeftClimb).withName("MiddleLeftClimb"));
        autoChooser.addOption("rightClimb", autoWrapper(this::rightClimb).withName("RightClimb"));
        autoChooser.addOption("leftClimb", autoWrapper(this::leftClimb).withName("LeftClimb"));
        autoChooser.onChange(c -> {
            switch (c.getName()) {
                case "Do Nothing":
                    break;
                case "RightShootClimb":
                    autoWrapper(this::rightTrifecta);
                    break;
                case "RightRefilTrifecta":
                    autoWrapper(this::rightRefilTrifecta);
                    break;
                case "LeftShootClimb":
                    autoWrapper(this::leftTrifecta);
                    break;
                case "RightAllOfTheMarbles":
                    autoWrapper(this::rightCollectShootClimb);
                    break;
                case "RightShoot":
                    autoWrapper(this::rightShoot);
                    break;
                case "MiddleShoot":
                    autoWrapper(this::middleShoot);
                    break;
                case "MiddleShootRightClimb":
                    autoWrapper(this::middleShootRightClimb);
                    break;
                case "MiddleShootLeftClimb":
                    autoWrapper(this::middleShootLeftClimb);
                    break;
                case "MiddleRightClimb":
                    autoWrapper(this::middleRightClimb);
                    break;
                case "MiddleLeftClimb":
                    autoWrapper(this::middleLeftClimb);
                    break;
                case "RightClimb":
                    autoWrapper(this::rightClimb);
                    break;
                case "LeftClimb":
                    autoWrapper(this::leftClimb);
                    break;
                case "InvertedRightRefilTrifecta":
                    autoWrapper(this::invertedRightRefilTrifecta);
                    break;
                default:
                    break;
            }
        });
    }

    /**
     * Gets the selected autonomous command from the auto chooser.
     * 
     * @return the selected autonomous command, or null if no chooser is available
     */
    public Command getAutonomousCommand() {
        if (autoChooser == null) {
            // If the chooser wasn't created (e.g. PathPlanner misconfigured), return a safe no-op
            return Commands.none();
        }
        return autoChooser.getSelected();
    }

    /**
     * Returns an example autonomous command.
     * 
     * @return a PathPlannerAuto command for "Dummy1"
     */
    public Command example_auto() {
        return new PathPlannerAuto("Dummy1");
    }
    
    /**
     * Creates a right shoot auto command
     * 
     * @return the right shoot auto command
     */
    public Command rightShoot() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("RightStart_RightShoot");
        
        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1);

        return Commands.sequence(
            resetOdom(p1),
            pathPlannerDtpPath(p1),
            namedCommands.get("Shoot").get().withTimeout(6)
        );
    }

    /**
     * Creates a right collect shoot climb auto command
     * If we ever use this, I might cry tears of joy and go into shock.
     * 
     * @return the right collect shoot climb auto command
     */
    public Command rightCollectShootClimb() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("RightStart_RightMid");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("RightCollectBallsPath");
        PathPlannerPath p3 = PathPlannerPath.fromChoreoTrajectory("RightBalls_RightMid");
        PathPlannerPath p4 = PathPlannerPath.fromChoreoTrajectory("RightMid_RightStart");
        PathPlannerPath p5 = PathPlannerPath.fromChoreoTrajectory("RightStart_RightShoot");
        PathPlannerPath p6 = PathPlannerPath.fromChoreoTrajectory("RightShoot_RightClimb");

        return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                Commands.deadline(
                        namedCommands.get("Intake").get(),
                        pathPlannerDtpPath(p2)).withTimeout(5),
                pathPlannerDtpPath(p3),
                pathPlannerDtpPath(p4),
                pathPlannerDtpPath(p5),
                namedCommands.get("AimShoot").get().withTimeout(5),
                Commands.parallel(
                        pathPlannerDtpPath(p6),
                        namedCommands.get("ClimberUp").get()),
                namedCommands.get("ClimberDown").get());
    }

    /**
     * Command to drive to the left for aligning onto the ladder.
     * 
     * @return the drive to left command
     */
    public Command adjustLeft() {
        return drive_subsystem.driveWithChassisSpeedsSupplier(
                SwerveInputStream.of(drive_subsystem.getSwerveDrive(), () -> -0.0, () -> 0.2)
                        .allianceRelativeControl(true).withControllerHeadingAxis(() -> 1, () -> 0));
    }

    /**
     * Command to drive to the right for aligning onto the ladder.
     * 
     * @return the drive to right command
     */
    public Command adjustRight() {
        return drive_subsystem.driveWithChassisSpeedsSupplier(
                SwerveInputStream.of(drive_subsystem.getSwerveDrive(), () -> -0.0, () -> -0.2)
                        .allianceRelativeControl(true).withControllerHeadingAxis(() -> 1, () -> 0));
    }

    /**
     * Creates a auto that refills, shoots, and climbs.
     * 
     * @return the auto
     */
    public Command rightRefilTrifecta() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("RightStart_HumanPlayerStation");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("HumanPlayerStation_RightShoot");
        PathPlannerPath p3 = PathPlannerPath.fromChoreoTrajectory("RightShoot_RightClimb");

        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2, p3);

        return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                new WaitCommand(1),
                AutoBuilder.followPath(p2),
                namedCommands.get("AimShoot").get().withTimeout(7),
                Commands.parallel(
                        pathPlannerDtpPath(p3),
                        namedCommands.get("ClimberUp").get()),
                adjustLeft().withTimeout(0.6),
                namedCommands.get("ClimberDown").get());
    }

    /**
     * Creates a auto that refills, shoots, and climbs.
     * 
     * @return the auto
     */
    public Command invertedRightRefilTrifecta() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("InvertedRightStart_HumanPlayerStation");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("HumanPlayerStation_RightShoot");
        PathPlannerPath p3 = PathPlannerPath.fromChoreoTrajectory("RightShoot_RightClimb");

        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2, p3);

        return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                new WaitCommand(1),
                AutoBuilder.followPath(p2),
                namedCommands.get("AimShoot").get().withTimeout(7),
                Commands.parallel(
                        pathPlannerDtpPath(p3),
                        namedCommands.get("ClimberUp").get()),
                adjustLeft().withTimeout(0.6),
                namedCommands.get("ClimberDown").get());
    }

    /**
     * An auto that shoots and climbs.
     * 
     * @return the auto
     * @throws Exception (I'm not sure which one) if the path fails to be made
     */
    public Command rightClimb() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("RightStart_RightShoot");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("RightShoot_RightClimb");

        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2);

        return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                Commands.parallel(
                        pathPlannerDtpPath(p2),
                        namedCommands.get("ClimberUp").get()),
                adjustLeft().withTimeout(0.6),
                namedCommands.get("ClimberDown").get());
    }

    /**
     * An auto that shoots and climbs.
     * 
     * @return the auto
     */
    public Command rightTrifecta() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("RightStart_RightShoot");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("RightShoot_RightClimb");

        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2);

        return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                namedCommands.get("Shoot").get().withTimeout(6.0),
                Commands.parallel(
                        pathPlannerDtpPath(p2),
                        namedCommands.get("ClimberUp").get()),
                adjustLeft().withTimeout(0.6),
                namedCommands.get("ClimberDown").get());
    }

    /**
     * Resets the odometry of the robot to the starting pose of a path.
     * 
     * @param path the path to reset the odometry to
     * @return the command to reset the odometry
     */
    public Command resetOdom(PathPlannerPath path) {
        return Commands.runOnce(() -> {
            if (Dashboard.getAlliance() == Alliance.Blue) {
                drive_subsystem.resetOdometry(path.getStartingHolonomicPose().get());
            } else {
                drive_subsystem.resetOdometry(path.flipPath().getStartingHolonomicPose().get());
            }
        });
    }

    /**
     * Creates a middle shoot auto command
     * 
     * @return the middle shoot auto command
     */
    public Command middleShoot() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("MidStart_MidShoot");

        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1);

        return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                namedCommands.get("AimShoot").get().withTimeout(6));
    }
    /**
     * Creates a middle climb shoot auto command
     * 
     * @return the middle climb shoot auto command
     */
    public Command middleShootRightClimb() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("MidStart_MidShoot");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("MidShoot_RightClimb");
        
        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2);
        
        return Commands.sequence(
            resetOdom(p1),
            pathPlannerDtpPath(p1),
            namedCommands.get("AimShoot").get().withTimeout(6),
            Commands.parallel(
                namedCommands.get("ClimberUp").get(),
                pathPlannerDtpPath(p2)
            ),
            adjustLeft().withTimeout(0.6),
            namedCommands.get("ClimberDown").get()
        );
    }

    /**
     * Creates a middle climb shoot auto command
     * 
     * @return the middle climb shoot auto command
     */
    public Command middleShootLeftClimb() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("MidStart_MidShoot");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("MidShoot_LeftClimb");
        
        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2);
        
        return Commands.sequence(
            resetOdom(p1),
            pathPlannerDtpPath(p1),
            namedCommands.get("AimShoot").get().withTimeout(6),
            Commands.parallel(
                namedCommands.get("ClimberUp").get(),
                pathPlannerDtpPath(p2)
            ),
            adjustRight().withTimeout(0.6),
            namedCommands.get("ClimberDown").get()
        );
    }
    /**
     * Middle start to right climb (no shooting)
     */
    public Command middleRightClimb() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("MidStart_MidShoot");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("MidShoot_RightClimb");

        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2);

        return Commands.sequence(
            resetOdom(p1),
            pathPlannerDtpPath(p1),
            Commands.parallel(
                pathPlannerDtpPath(p2),
                namedCommands.get("ClimberUp").get()
            ),
            adjustLeft().withTimeout(0.6),
            namedCommands.get("ClimberDown").get()
        );
    }

    /**
     * Middle start to left climb (no shooting)
     */
    public Command middleLeftClimb() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("MidStart_MidShoot");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("MidShoot_LeftClimb");

        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2);

        return Commands.sequence(
            resetOdom(p1),
            pathPlannerDtpPath(p1),
            Commands.parallel(
                pathPlannerDtpPath(p2),
                namedCommands.get("ClimberUp").get()
            ),
            adjustRight().withTimeout(0.6),
            namedCommands.get("ClimberDown").get()
        );
    }
    /**
     * An auto that shoots and climbs.
     * 
     * @return the auto
     */
    public Command leftTrifecta() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("LeftStart_LeftShoot");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("LeftShoot_LeftClimb");
        
        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2);
        
        return Commands.sequence(
            resetOdom(p1),
            pathPlannerDtpPath(p1),
            namedCommands.get("AimShoot").get().withTimeout(5),
            namedCommands.get("ClimberUp").get(),
            Commands.parallel(
                pathPlannerDtpPath(p2)
                
            ),
            adjustRight().withTimeout(0.6),
            namedCommands.get("ClimberDown").get()
        );
    }

    /**
     * An auto that shoots and climbs.
     * 
     * @return the auto
     */
    public Command leftClimb() throws Exception {
        PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("LeftStart_LeftShoot");
        PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("LeftShoot_LeftClimb");
        
        // Build trajectory for visualization
        getTrajectoryOfCombinedPaths(p1, p2);
        
        return Commands.sequence(
            resetOdom(p1),
            pathPlannerDtpPath(p1),
            namedCommands.get("ClimberUp").get(),
            Commands.parallel(
                pathPlannerDtpPath(p2)
                
            ),
            adjustRight().withTimeout(0.6),
            namedCommands.get("ClimberDown").get()
        );
    }
    /**
     * Gets a trajectory from a list of {@link PathPlannerPath}s and logs it to the Dashboard.
     * 
     * @param paths the paths to get the trajectory from
     * @return the trajectory
     */
    public Trajectory getTrajectoryOfCombinedPaths(PathPlannerPath ...paths) {
        List<State> combinedStates = new ArrayList<>();
        for (PathPlannerPath path : paths) {
            if (Dashboard.getAlliance() == Alliance.Red) {
                path = path.flipPath();
            }
            for (Pose2d pose: path.getPathPoses()) {
                State state = new State(0, 0, 0, pose, 0);
                combinedStates.add(state);
            }
        }
        Trajectory combined = new Trajectory(combinedStates);
        
        // Automatically publish the trajectory to the field for visualization
        Dashboard.getField2d().getObject("traj").setTrajectory(combined);
        
        return combined;
    }
}
