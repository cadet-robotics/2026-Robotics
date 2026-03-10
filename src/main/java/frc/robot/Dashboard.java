// A helper class to manage the dashboard and the values that will be displayed on it.
package frc.robot;

import java.util.Optional;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {
    public static NetworkTable dashboardTable = NetworkTableInstance.getDefault().getTable("Dashboard");
    private static BooleanSubscriber manualOverrideReciever = dashboardTable.getBooleanTopic("AutonomousOverride").subscribe(false);
    private static StringSubscriber allianceBackupSelector = dashboardTable.getStringTopic("AllianceBackup").subscribe("Blue");
    private static BooleanPublisher elevatorDownPublisher = dashboardTable.getBooleanTopic("DriveUnderTrench").publish();
    private static BooleanPublisher hubActivePublisher = dashboardTable.getBooleanTopic("HubActive").publish();
    private static StringPublisher matchPhasePublisher = dashboardTable.getStringTopic("MatchPhase").publish();
    private static DoublePublisher matchPhaseChangePublisher = dashboardTable.getDoubleTopic("PhaseChangeIn").publish();
    private static DoublePublisher shootRangeOffsetPublisher = dashboardTable.getDoubleTopic("ShootRangeOffset").publish();
    private static DoubleSubscriber shootRangeOffsetReciever = dashboardTable.getDoubleTopic("ShootRangeOffset").subscribe(0.0);
    private static Field2d field = new Field2d();

    /**
     * Gets the manual override state.
     * 
     * @return the manual override state
     */
    public static boolean getManualOverride() {
        return manualOverrideReciever.get();
    }

    /**
     * Gets the shoot range offset which is used to offset the shooter's drive to curve radius.
     * 
     * @return the shoot range offset -1.0 to 1.0
     */
    public static double getShootRangeOffset() {
        return shootRangeOffsetReciever.get();
    }  

    public static void isElevatorDown(boolean isDown) {
        if (isDown == true) {
            elevatorDownPublisher.set(true);
        } else {
            elevatorDownPublisher.set(false);
        }
    }

    public static void field2dInit() {
        SmartDashboard.putData("Dashboard/Field", field);
    }

    public static Field2d getField2d() {
        return field;
    }
    
    /**
     * Gets the alliance of the robot safely, falling back to the alliance backup selector.
     * 
     * @return the alliance of the robot
     */
    public static Alliance getAlliance() {
        Optional<Alliance> shrodingersAlliance = DriverStation.getAlliance();
        if ( shrodingersAlliance.isEmpty() ) {
            if (allianceBackupSelector.get().equals("Blue")) {
                return Alliance.Blue;
            } else if (allianceBackupSelector.get().equals("Red")) {
                return Alliance.Red;
            } else {
                return Alliance.Blue;
            }
        }
        return shrodingersAlliance.get();
    }

    /**
     * Handles the match state changes based on the match time and who won the autonomous section.
     */
    public static void matchPhaseChange() {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime > 140) {
            // Autonomous, 20 seconds until next shift
            matchPhaseChangePublisher.set(matchTime - 140);
            matchPhasePublisher.set("Autonomous");
            return;
        } else if (140 >= matchTime && matchTime > 130) {
            // Transition shift, 10 seconds until shift 1
            matchPhaseChangePublisher.set(matchTime - 130);
            matchPhasePublisher.set("Transition");
            return;
        } else if (130 >= matchTime && matchTime > 105) {
            // Shift 1, 25 seconds until shift 2
            matchPhaseChangePublisher.set(matchTime - 105); 
            matchPhasePublisher.set("Shift 1");
            return;
        } else if (105 >= matchTime && matchTime > 80) {
            // Shift 2, 25 seconds until shift 3
            matchPhaseChangePublisher.set(matchTime - 80);
            matchPhasePublisher.set("Shift 2");
            return;
        } else if (80 >= matchTime && matchTime > 55) {
            // Shift 3, 25 seconds until shift 4
            matchPhaseChangePublisher.set(matchTime - 55);
            matchPhasePublisher.set("Shift 3");
            return;
        } else if (55 >= matchTime && matchTime > 30) {
            // Shift 4, 25 seconds until endgame
            matchPhaseChangePublisher.set(matchTime - 30);
            matchPhasePublisher.set("Shift 4");
            return;
        } else if (30 >= matchTime && matchTime > 0) {
            // Endgame, 30 seconds until game ends
            matchPhaseChangePublisher.set(matchTime);
            matchPhasePublisher.set("Endgame!");
            return;
        } else {
            // Outside of match
            matchPhasePublisher.set("Outside of match");
            return;
        }
    }

    /**
     * Returns wither the hub is active or not based on the alliance, match time, and the FMS game data.
     *
     * @return wither the hub is active or not
     */
    public static void isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            hubActivePublisher.set(false);
            return; }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            hubActivePublisher.set(true);
            return; }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            hubActivePublisher.set(false);
            return; }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            hubActivePublisher.set(true);
            return; }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                hubActivePublisher.set(true);
                return; } }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst; };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            hubActivePublisher.set(true);
            return;
        } else if (matchTime > 105) {
            // Shift 1
            hubActivePublisher.set(shift1Active);
            return;
        } else if (matchTime > 80) {
            // Shift 2
            hubActivePublisher.set(!shift1Active);
            return;
        } else if (matchTime > 55) {
            // Shift 3
            hubActivePublisher.set(shift1Active);
            return;
        } else if (matchTime > 30) {
            // Shift 4
            hubActivePublisher.set(!shift1Active);
            return;
        } else if (matchTime > 0) {
            // End game, hub always active.
            hubActivePublisher.set(true); 
        } else {
            // If we have negative time, we're likely outside of a match, assume hub is active.
            hubActivePublisher.set(true);
        }
    }
}
