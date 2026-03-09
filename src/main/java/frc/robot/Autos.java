package frc.robot; 

import java.nio.file.Path;
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
import frc.robot.Libs.CCommand;
import frc.robot.Subsystems.Drive;
import swervelib.SwerveInputStream;
import swervelib.encoders.SwerveAbsoluteEncoder;

/**
 * Class for managing autonomous routines.
 * Provides auto selection through a SendableChooser and individual auto commands.
 */
public class Autos {
    private SendableChooser<Command> autoChooser;
    private HashMap<String, Supplier<Command>> namedCommands = new HashMap<>();
    private HashMap<String, PathPlannerPath> paths = new HashMap<>();
    private HashMap<String, Trajectory> trajectories = new HashMap<>();
    // Track the last selection from the SendableChooser so we only recompute when it changes
    private Command lastSelectedCommand = null;

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

    public Pose2d getStartingPoseFromPath(PathPlannerPath path) {
        if (Dashboard.getAlliance() == Alliance.Blue ) {
            return path.getStartingHolonomicPose().get();
        } else {
            return path.flipPath().getStartingHolonomicPose().get();
        }
    }

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

    public Command pathPlannerDtpPath(PathPlannerPath path) {
        return Commands.sequence(
            drive_subsystem.driveToTargetPose(getStartingPoseFromPath(path), 0),
            AutoBuilder.followPath(path),
            drive_subsystem.driveToTargetPose(getEndPoseFromPath(path), 0)
        );
    }

    public void addCommand( String name, Supplier<Command> command) {
        namedCommands.put(name, command);
    }

    public void addAutos() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("RightShootClimb", rightTrifecta());
        autoChooser.addOption("RightRefilTrifecta", rightRefilTrifecta());
        autoChooser.addOption("LeftShootClimb", leftTrifecta());
        autoChooser.addOption("RightAllOfTheMarbles", rightCollectShootClimb());
        autoChooser.addOption("RightShoot", rightShoot());
        autoChooser.addOption("MiddleShoot", middleShoot());
    }

    /**
     * Gets the selected autonomous command from the auto chooser.
     * 
     * @return the selected autonomous command, or null if no chooser is available
     */
    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("Test1");
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
    
    public Command rightShoot() {
        try {
            PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("RightStart_RightShoot");
            
            // Build trajectory for visualization
            getTrajectoryOfCombinedPaths(p1);

            return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                namedCommands.get("Shoot").get().withTimeout(6)
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command rightCollectShootClimb() {
        try {
            PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("RightStart_RightMid");
            PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("RightCollectBallsPath");
            PathPlannerPath p3 = PathPlannerPath.fromChoreoTrajectory("RightBalls_RightMid");
            PathPlannerPath p4 = PathPlannerPath.fromChoreoTrajectory("RightMid_RightStart");
            PathPlannerPath p5 = PathPlannerPath.fromChoreoTrajectory("RightStart_RightShoot");
            PathPlannerPath p6 = PathPlannerPath.fromChoreoTrajectory("RightShoot_RightClimb");

            getTrajectoryOfCombinedPaths(p1, p2, p3, p4, p5, p6);
            if (Dashboard.getAlliance() == Alliance.Blue ) {
                drive_subsystem.resetOdometry(p1.getStartingHolonomicPose().get());
            } else {
                drive_subsystem.resetOdometry(p1.flipPath().getStartingHolonomicPose().get());
            }

            return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                Commands.deadline(
                    namedCommands.get("Intake").get(),
                    pathPlannerDtpPath(p2)
                ).withTimeout(5), 
                pathPlannerDtpPath(p3),
                pathPlannerDtpPath(p4),
                pathPlannerDtpPath(p5),
                namedCommands.get("AimShoot").get().withTimeout(5),
                Commands.parallel(
                    pathPlannerDtpPath(p6),
                    namedCommands.get("ClimberUp").get()
                ),
                namedCommands.get("ClimberDown").get()
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return new PathPlannerAuto("Test1");
        }
    }

    public Command adjustLeft() {
        return drive_subsystem.driveWithChassisSpeedsSupplier( SwerveInputStream.of(drive_subsystem.getSwerveDrive(), ()->-0.0,()->0.2).allianceRelativeControl(true).withControllerHeadingAxis(()->1,()->0));
    }

    public Command adjustRight() {
        return drive_subsystem.driveWithChassisSpeedsSupplier( SwerveInputStream.of(drive_subsystem.getSwerveDrive(), ()->-0.0,()->-0.2).allianceRelativeControl(true).withControllerHeadingAxis(()->1,()->0));
    }
    
    public Command rightRefilTrifecta() {
        try {
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
                    namedCommands.get("ClimberUp").get()
                ),
                adjustLeft().withTimeout(0.6),
                namedCommands.get("ClimberDown").get()
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return new PathPlannerAuto("Test1");
        }
    }


    public Command rightTrifecta() {
        try {
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
                    namedCommands.get("ClimberUp").get()
                ),
                adjustLeft().withTimeout(0.6),
                namedCommands.get("ClimberDown").get()
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return new PathPlannerAuto("Test1");
        }
    }

    public Command resetOdom(PathPlannerPath path) {
        return Commands.runOnce(() -> {
            if (Dashboard.getAlliance() == Alliance.Blue ) {
                drive_subsystem.resetOdometry(path.getStartingHolonomicPose().get());
            } else {
                drive_subsystem.resetOdometry(path.flipPath().getStartingHolonomicPose().get());
            }
        });
    }

    public Command middleShoot() {
        try {
            PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("MidStart_MidShoot");
            
            // Build trajectory for visualization
            getTrajectoryOfCombinedPaths(p1);
            
            return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                namedCommands.get("AimShoot").get().withTimeout(6)
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return new PathPlannerAuto("Test1");
        }
        
    }
    public Command leftTrifecta() {
        try {
            PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("LeftStart_LeftShoot");
            PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("LeftShoot_LeftClimb");
            
            // Build trajectory for visualization
            getTrajectoryOfCombinedPaths(p1, p2);
            
            return Commands.sequence(
                resetOdom(p1),
                pathPlannerDtpPath(p1),
                namedCommands.get("AimShoot").get().withTimeout(3),
                namedCommands.get("ClimberUp").get(),
                Commands.parallel(
                    pathPlannerDtpPath(p2)
                    
                ),
                namedCommands.get("ClimberDown").get()
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return new PathPlannerAuto("Test1");
        }
    }

    public Trajectory getTrajectoryOfCombinedPaths(PathPlannerPath ...paths) {
        List<State> combinedStates = new ArrayList<>();
        for (PathPlannerPath path : paths) {
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
