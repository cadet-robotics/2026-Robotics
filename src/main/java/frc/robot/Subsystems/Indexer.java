package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IndexerSubsystemConstants;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Indexer subsystem that moves game pieces between the intake and shooter.
 * Controls a motor to transfer game pieces in both directions based on shooter and intake states.
 */
public class Indexer extends CSubsystem {
    /** Motor controller for the indexer mechanism. */
    private final SparkFlex indexerMotorController = new SparkFlex(1, SparkLowLevel.MotorType.kBrushless);
    /** Configuration for the smart motor controller including PID, feedforward, and gearing. */
    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("IndexerMotor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
        // You could also use .withGearing(12) which does the same thing.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    /** Smart motor controller wrapper for the indexer motor. */
    private final SmartMotorController indexerController = new SparkWrapper(
            indexerMotorController,
            DCMotor.getNeoVortex(1),
            smcConfig
    );

    /** SysId routine for motor characterization. */
    private final SysIdRoutine sysIdRoutine;

    /** Reference to the shooter subsystem. */
    private Shooter shooterSubsystem;
    /** Reference to the intake subsystem. */
    private Intake intakeSubsystem;

    /**
     * Represents the operational state of the indexer.
     */
    private static enum IndexerState {
        /** Indexing towards the shooter. */
        Shooter,
        /** Indexing towards the hopper/storage. */
        Hopper,
        /** Indexer is off. */
        Off
    }
    /** Current state of the indexer. */
    private IndexerState indexerState= IndexerState.Off;

    /**
     * Constructs a new Indexer subsystem.
     * Initializes motor controllers, SysId routine, and sets up default command.
     * 
     * @param shooterSubsystem the shooter subsystem instance
     * @param intakeSubsystem the intake subsystem instance
     */
    public Indexer( Shooter shooterSubsystem, Intake intakeSubsystem ) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        
        // Initialize SysId routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> indexerController.setVoltage(volts),
                null, // No log consumer (can add if needed)
                this
            )
        );
        
        // Register SysId commands with SmartDashboard
        SmartDashboard.putData("Indexer/SysId Quasistatic Forward", 
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Indexer/SysId Quasistatic Reverse", 
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Indexer/SysId Dynamic Forward", 
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Indexer/SysId Dynamic Reverse", 
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        
        setDefaultCommand(indexerHandler());
    }

    /**
     * Creates a command to run the indexer towards the shooter.
     * 
     * @return command that sets indexer state to Shooter
     */
    public CCommand shooterIndexing() {
        return cCommand( "ShootingIndexing")
                .onInitialize(() -> indexerState = Indexer.IndexerState.Shooter);
    }

    /**
     * Creates a command to run the indexer towards the hopper/storage.
     * 
     * @return command that sets indexer state to Hopper
     */
    public CCommand hopperIndexing() {
        return cCommand( "HopperIndexing")
                .onInitialize(() -> indexerState = Indexer.IndexerState.Hopper);
    }

    /**
     * Creates a command to stop the indexer.
     * 
     * @return command that sets indexer state to Off
     */
    public CCommand stopIndexer() {
        return cCommand( "StopIndexer")
                .onInitialize(() -> indexerState = Indexer.IndexerState.Off);
    }

    /**
     * Creates the default command that automatically controls the indexer based on subsystem states.
     * Runs towards shooter when shooter is on, towards hopper when intake is on, otherwise stops.
     * 
     * @return command that handles automatic indexer control
     */
    public CCommand indexerHandler() {
        return cCommand("IndexerHandler")
                .onExecute(() -> {
                   if ( shooterSubsystem.getState() == Shooter.ShooterState.On ) {
                       indexerState = Indexer.IndexerState.Shooter;
                       indexerController.setVelocity(IndexerSubsystemConstants.forwardsOnSpeeds);
                   } else if ( intakeSubsystem.getState() == Intake.IntakeState.On ) {
                       indexerState = Indexer.IndexerState.Hopper;
                       indexerController.setVelocity(IndexerSubsystemConstants.backwardsOnSpeeds);
                   } else {
                       indexerState = Indexer.IndexerState.Off;
                       indexerController.setVelocity(RPM.of(0));
                   }
                });
    }

    /**
     * Updates telemetry data for the indexer motor controller.
     * Called periodically by the command scheduler.
     */
    @Override
    public void periodic() {
        indexerController.updateTelemetry();
    }

    /**
     * Iterates the motor controller simulation.
     * Called periodically during simulation mode.
     */
    @Override
    public void simulationPeriodic() {
        indexerController.simIterate();
    }
}