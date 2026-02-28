// https://github.com/Greater-Rochester-Robotics/GRRBase/blob/main/src/main/java/org/team340/lib/util/command/GRRSubsystem.java
package frc.robot.Libs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CSubsystem extends SubsystemBase {
    // Constructor - SubsystemBase handles registration
    public CSubsystem() {
        super();
    }

    // Creating a new command
    public CCommand cCommand() {
        return new CCommand( this );
    }

    // Creating a new command with a name
    public CCommand cCommand( String name ) {
        return new CCommand( name, this );
    }

    public void logSelf() {
        SmartDashboard.putData("Subsystems/" + this.getName(), this);
    }
}