package frc.robot.Subsystems;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IndexerState;
import frc.robot.Libs.CSubsystem;

public class Shaker extends CSubsystem {
    private SparkMax shaker_motor = new SparkMax(30, MotorType.kBrushless);

    private ShakerState state = ShakerState.Off;
    private IndexerState cached_indexerState = IndexerState.OFF;
    private Supplier<IndexerState> indexerStateSupplier;

    public Shaker( Indexer indexer_subsystem ) {
        setName("ShakerSubsystem");

        indexerStateSupplier = indexer_subsystem::getState;
    }

    public ShakerState getState() {
        return this.state;
    }

    @Override 
    public void periodic() {
        logSelf();

        IndexerState incomming_state = indexerStateSupplier.get();
        if (cached_indexerState != incomming_state) {
            switch (incomming_state) {
                case OFF: 
                case HOPPER:
                case MANUAL_BACKWARD:
                case MANUAL_FORWARD:
                    state = ShakerState.Off;
                    shaker_motor.set(0);
                    break; 
                case SHOOTER:
                    state = ShakerState.On;
                    shaker_motor.set(0.5);
                    break;
            }
            cached_indexerState = incomming_state;
        }
    }
    
    public static enum ShakerState {
        On,
        Off
    }
}
