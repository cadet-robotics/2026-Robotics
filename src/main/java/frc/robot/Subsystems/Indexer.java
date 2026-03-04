package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.IndexerState;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ShooterState;
import frc.robot.Constants.RobotConstants;

/**
 * Indexer subsystem that moves game pieces between the intake and shooter.
 * Controls a motor to transfer game pieces in both directions based on shooter and intake states.
 */
public class Indexer extends CSubsystem {
    /** Motor controller for the indexer mechanism. */
    private final SparkMax indexerMotorController = new SparkMax(20, SparkLowLevel.MotorType.kBrushless);

    /** Reference to the shooter subsystem. */
    private Shooter shooterSubsystem;
    /** Reference to the intake subsystem. */
    private Intake intakeSubsystem;
    /** Reference to drive subsystem for gating while-shooting behavior. */
    private Drive driveSubsystem;

    /** Current state of the indexer. */
    private IndexerState indexerState = IndexerState.OFF;

    /**
     * Constructs a new Indexer subsystem.
     * 
     * @param shooterSubsystem the shooter subsystem instance
     * @param intakeSubsystem the intake subsystem instance
     */
    public Indexer( Shooter shooterSubsystem, Intake intakeSubsystem, Drive driveSubsystem ) {
        setName("IndexerSubsystem");

        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.driveSubsystem = driveSubsystem;
    }

    /**
     * Creates a command to stop the indexer.
     * 
     * @return command that sets indexer state to Off
     */
    public CCommand stopIndexer() {
        return cCommand( "StopIndexer")
            .onInitialize(() -> indexerState = IndexerState.OFF);
    }

    public CCommand IndexerOut() {
        return cCommand("IndexerOut")
            .onInitialize(() -> {
                indexerState = IndexerState.SHOOTER;
                indexerMotorController.setVoltage(9);
            })
            .onEnd(() -> {
                indexerState = IndexerState.OFF;
                indexerMotorController.setVoltage(0);
            });
    }

    public CCommand IndexerIn() {
        return cCommand("IndexIn")
            .onInitialize(() -> {
                indexerState = IndexerState.HOPPER;
                indexerMotorController.setVoltage(-9);
            })
            .onEnd(() -> {
                indexerState = IndexerState.OFF;
                indexerMotorController.setVoltage(-9);
            });
    }

    public IndexerState getState() {
        return indexerState;
    }    

    @Override
    public void periodic() {
        logSelf();
    }
}