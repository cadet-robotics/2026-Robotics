package frc.robot.Configuration;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.RobotConstants.DriveSubsystemConstants;
import frc.robot.Subsystems.Drive;
import swervelib.SwerveDrive;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public final class DriveSubsystemConfiguration {

    public static void configurePathPlanner(Drive driveSubsystem, SwerveDrive swerveDrive ) {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // If config file loading fails, report and return without configuring
            DriverStation.reportError(
                    "Failed to load PathPlanner config: " + e.getMessage(),
                    e.getStackTrace()
            );
            return;
        }

        BooleanSupplier isRedTeam = () -> {
            Optional< DriverStation.Alliance > alliance = DriverStation.getAlliance();
            if ( alliance.isPresent() ) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

        // Set the pathfinding implementation to use ADStar
        Pathfinding.setPathfinder(new com.pathplanner.lib.pathfinding.LocalADStar());

        AutoBuilder.configure(
            swerveDrive::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            (ChassisSpeeds speedsRobotRelative, DriveFeedforwards moduleFeedForwards ) -> {
                swerveDrive.drive(
                        speedsRobotRelative,
                        swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces()
                );
            },
            new PPHolonomicDriveController(
                    DriveSubsystemConstants.pptranslationPidConstants,
                    DriveSubsystemConstants.pprotationPidConstants
            ),
            config,
            isRedTeam,
            driveSubsystem
        );
    }
}        