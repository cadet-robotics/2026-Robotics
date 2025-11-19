package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystems.Drive;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class Configs {
    public static final class DriveSubsystem {

        public static void configurePathPlanner( Drive driveSubsystem, SwerveDrive swerveDrive, AutoBuilder autoBuilder ) {
            if ( true ) {
                return;
            }
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
                            "Unable to parse pathplanner config file " + e.getLocalizedMessage(),
                            printStackTrace
                    );
                }
                throw new RuntimeException(e);
            }

            BooleanSupplier isRedTeam = () -> {
                Optional< DriverStation.Alliance > alliance = DriverStation.getAlliance();
                if ( alliance.isPresent() ) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            };

            autoBuilder.configure(
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
                        Constants.DriveSubsystem.translationPidConstants,
                        Constants.DriveSubsystem.rotationPidConstants
                ),
                config,
                isRedTeam,
                driveSubsystem
            );
        }
    }
}