package frc.robot; 

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drive;

public class Autos {

   private final AutoBuilder autoBuilder;
   private final Drive driveSubsystem;

   public Autos( RobotContainer robotContainer, Drive driveSubsystem ) {
       this.driveSubsystem = driveSubsystem;

       // Instead of Choreo, import Choreo paths into pathplanner

       // Set up the Choreo AutoFactor
       // Guide to using the AutoFactory https://choreo.autos/choreolib/auto-factory/
       // this.autoFactory = new AutoFactory(
       //         driveSubsystem::getPose,
       //         driveSubsystem::resetOdometry,
       //         driveSubsystem::followTrajectory,
       //         true,
       //         driveSubsystem
       // );
       // this.autoChooser = new AutoChooser();
       this.autoBuilder = driveSubsystem.getAutoBuilder();

   }

   // public Command getAutonomousCommand() {
   //   return this.autoBuilder.sel();
   // }

   // public Command example_auto() {
   //   this.autoBuilder.
   // }
}
