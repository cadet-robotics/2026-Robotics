package frc.robot; 

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive;

/**
 * Class for managing autonomous routines.
 * Provides auto selection through a SendableChooser and individual auto commands.
 */
public class Autos {
    private SendableChooser<Command> autoChooser;
    private HashMap<String, Command> namedCommands = new HashMap<>();
    private HashMap<String, PathPlannerPath> paths = new HashMap<>();
    private HashMap<String, Trajectory> trajectories = new HashMap<>();
    // Track the last selection from the SendableChooser so we only recompute when it changes
    private Command lastSelectedCommand = null;

    /**
     * Constructs the Autos object and initializes the auto chooser.
     * 
     * @param robotContainer the robot container instance
     * @param driveSubsystem the drive subsystem instance
     */
    public Autos( RobotContainer robotContainer, Drive driveSubsystem ) {
        // AutoBuilder might not be configured if PathPlanner config has errors
        try {
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);
        } catch (RuntimeException e) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "Could not build auto chooser - PathPlanner may not be configured: " + e.getMessage(), 
                false
            );
            autoChooser = null;
        }
    }

    public void addCommand( String name, Command command) {
        namedCommands.put(name, command);
    }

    /**
     * Gets the selected autonomous command from the auto chooser.
     * 
     * @return the selected autonomous command, or null if no chooser is available
     */
    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("Test1");
        return leftTrifecta();
    }

    /**
     * Returns an example autonomous command.
     * 
     * @return a PathPlannerAuto command for "Dummy1"
     */
    public Command example_auto() {
        return new PathPlannerAuto("Dummy1");
    }

    public Command leftTrifecta() {
        try {
            PathPlannerPath p1 = PathPlannerPath.fromChoreoTrajectory("LeftStart_LeftShoot");
            PathPlannerPath p2 = PathPlannerPath.fromChoreoTrajectory("LeftShoot_LeftClimb");
            
            // Build trajectory for visualization
            getTrajectoryOfCombinedPaths(p1, p2);
            
            return Commands.sequence(
                AutoBuilder.followPath(p1),
                namedCommands.get("Shoot"),
                Commands.parallel(
                    AutoBuilder.followPath(p2),
                    namedCommands.get("ClimberUp")
                ),
                namedCommands.get("ClimberDown")
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return new PathPlannerAuto("Test1");
        }
    }

    public void disabledPath() {
        // Clear any preview by default
        Dashboard.getField2d().getObject("traj").setTrajectory(null);
    }

    /**
     * Called periodically while disabled to optionally update any heavy/autonomous preview work.
     * This will only actually run the disabledPath computation if the choice in the chooser changed
     * since the last call. Call this from Robot.disabledPeriodic (or via RobotContainer) each
     * disabled loop so work is only done on selection changes.
     */
    public void maybeUpdateDisabledPath() {
        if (autoChooser == null) {
            return;
        }
        Command selected = autoChooser.getSelected();
        if (selected == lastSelectedCommand) {
            // no change, do nothing
            return;
        }
        // selection changed; rebuild the auto command which will trigger trajectory building
        lastSelectedCommand = selected;
        
        // Call getAutonomousCommand to trigger the auto building (which calls getTrajectoryOfCombinedPaths)
        getAutonomousCommand();
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
