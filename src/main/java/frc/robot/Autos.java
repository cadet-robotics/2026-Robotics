package frc.robot; 

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drive;

public class Autos {

   private final AutoFactory autoFactory;
   private final AutoChooser autoChooser;
   private final Drive driveSubsystem;

   public Autos( RobotContainer robotContainer, Drive driveSubsystem ) {
      //this.driveSubsystem = robotContainer.driveSubsystem;
       this.driveSubsystem = driveSubsystem;
       // Set up the Choreo AutoFactor
       // Guide to using the AutoFactory https://choreo.autos/choreolib/auto-factory/
       this.autoFactory = new AutoFactory(
               driveSubsystem::getPose,
               driveSubsystem::resetOdometry,
               driveSubsystem::followTrajectory,
               true,
               driveSubsystem
       );
       this.autoChooser = new AutoChooser();
   }

   public Command getAutonomousCommand() {
      return this.autoChooser.selectedCommand();
   }
}
