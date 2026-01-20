package frc.robot; 

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive;

public class Autos {

    private SendableChooser<Command> autoChooser;

    public Autos( RobotContainer robotContainer, Drive driveSubsystem ) {
        // AutoBuilder might not be configured if PathPlanner config has errors
        try {
            this.autoChooser = AutoBuilder.buildAutoChooser();
        } catch (RuntimeException e) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "Could not build auto chooser - PathPlanner may not be configured: " + e.getMessage(), 
                false
            );
            this.autoChooser = null;
        }
    }

    public Command getAutonomousCommand() {
        if (this.autoChooser != null) {
            return this.autoChooser.getSelected();
        }
        return null;
    }

    public Command example_auto() {
        return new PathPlannerAuto("Dummy1");
    }
}
