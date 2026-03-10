package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;

public class Shaker extends CSubsystem {
    private SparkMax shaker_motor = new SparkMax(30, MotorType.kBrushless);

    private ShakerState state = ShakerState.OFF;

    public Shaker( Indexer indexer_subsystem ) {
        setName("ShakerSubsystem");
    }

    /** 
     * Gets the current state of the shaker (On or Off).
     * 
     * @return the current shaker state
     */
    public ShakerState getState() {
        return this.state;
    }

    public CCommand Shake() {
        return cCommand("Shake")
            .onInitialize(() -> {
                state = ShakerState.ON;
                shaker_motor.set(0.5);
            })
            .onEnd(() -> {
                state = ShakerState.OFF;
                shaker_motor.set(0);
            });
        }

    @Override 
    public void periodic() {
        logSelf();
    }
    
    public static enum ShakerState {
        ON,
        OFF
    }
}
