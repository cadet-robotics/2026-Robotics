package frc.robot.Configuration;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystems.Drive;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import frc.robot.Constants.DriveSubsystemConstants;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public final class DriveSubsystemConfiguration {

    public static void configurePathPlanner(Drive driveSubsystem, SwerveDrive swerveDrive ) {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e ) {
            boolean printStackTrace = true;
            if ( e instanceof IOException ) {
                DriverStation.reportError(
                        "Unable to read pathplanner config file: " + e.getLocalizedMessage(),
                        printStackTrace
                );
            } else if ( e instanceof ParseException ) {
                DriverStation.reportError(
                        "Unable to parse pathplanner config file: " + e.getLocalizedMessage(),
                        printStackTrace
                );
            }
            // Don't throw - just disable PathPlanner auto
            DriverStation.reportWarning("PathPlanner AutoBuilder not configured due to config file error", false);
            return;
        } catch (Exception e) {
            // Catch any other exceptions (like NullPointerException from missing fields)
            DriverStation.reportError(
                    "Unexpected error loading PathPlanner config: " + e.getLocalizedMessage(),
                    true
            );
            DriverStation.reportWarning("PathPlanner AutoBuilder not configured", false);
            return;
        }

        BooleanSupplier isRedTeam = () -> {
            Optional< DriverStation.Alliance > alliance = DriverStation.getAlliance();
            if ( alliance.isPresent() ) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

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
                    DriveSubsystemConstants.translationPidConstants,
                    DriveSubsystemConstants.rotationPidConstants
            ),
            config,
            isRedTeam,
            driveSubsystem
        );
    }
}        