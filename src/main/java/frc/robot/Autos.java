package frc.robot; 

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
            return Commands.sequence(
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("LeftStart_LeftShoot")),
                namedCommands.get("Shoot"),
                Commands.parallel(
                    AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("LeftShoot_LeftClimb")),
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
        Trajectory currentTrajectory = new Trajectory();
        Dashboard.getField2d().getObject("traj").setTrajectory(null);
    }
}
