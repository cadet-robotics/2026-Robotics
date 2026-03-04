// A helper class to manage the dashboard and the values that will be displayed on it.
package frc.robot;

import java.lang.reflect.Field;
import java.util.Optional;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {
    public static NetworkTable dashboardTable = NetworkTableInstance.getDefault().getTable("Dashboard");
    private static BooleanSubscriber manualOverrideReciever = dashboardTable.getBooleanTopic("AutonomousOverride").subscribe(false);
    private static StringSubscriber allianceBackupSelector = dashboardTable.getStringTopic("AllianceBackup").subscribe("Blue");
    private static BooleanPublisher elevatorUpPublisher = dashboardTable.getBooleanTopic("ElevatorUp").publish();
    private static Field2d field = new Field2d();

    public static boolean getManualOverride() {
        return manualOverrideReciever.get();
    }

    public static void setElevatorStatus(boolean isUp) {
        elevatorUpPublisher.set(isUp);
    }

    public static void field2dInit() {
        SmartDashboard.putData("Dashboard/Field", field);
    }

    public static Field2d getField2d() {
        return field;
    }
    
    public static Alliance getAlliance() {
        Optional<Alliance> shordingersAlliance = DriverStation.getAlliance();
        if ( shordingersAlliance.isEmpty() ) {
            if (allianceBackupSelector.get().equals("Blue")) {
                return Alliance.Blue;
            } else if (allianceBackupSelector.get().equals("Red")) {
                return Alliance.Red;
            } else {
                return Alliance.Blue;
            }
        }
        return shordingersAlliance.get();
    } 
}
