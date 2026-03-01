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
    /** Configuration for the smart motor controller including PID, feedforward, and gearing. */
    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(0.001, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("IndexerMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
        // You could also use .withGearing(12) which does the same thing.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4,4)))
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
        
        setDefaultCommand(indexerHandler());
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

    /**
     * Creates the default command that automatically controls the indexer based on subsystem states.
     * Runs towards shooter when shooter is on AND up to speed, towards hopper when intake is on, otherwise stops.
     * Uses voltage control: +11V to feed shooter, -11V to feed hopper, 0V when stopped.
     * 
     * @return command that handles automatic indexer control
     */
    public CCommand indexerHandler() {
        return cCommand("IndexerHandler")
                .onExecute(() -> {
                    // Get current states from subsystems
                    ShooterState shooterState = shooterSubsystem.getState();
                    IntakeState intakeState = intakeSubsystem.getState();
                    boolean shooterUpToSpeed = shooterSubsystem.isUpToSpeed();
                    
                    // Determine indexer action based on subsystem states
                    // Only feed shooter if it's on AND up to speed and any drive/aim gating passes
                    boolean allowedByDrive = true; // TODO: make aiming and autodriving regulate shooting
                    boolean allowedByAim = true;
                    if (driveSubsystem != null) {
                        if (driveSubsystem.isDriveToPoseActive()) {
                            // When autodrive is active, allow shooting only if the robot's distance
                            // to the hub is within a tolerance of the midRange used to generate the curve.
                            double midRange = RobotConstants.ShooterSubsystemConstants.midRange;
                            double posTol = midRange * 0.05; // 5% tolerance around midRange
                            double hubDist = driveSubsystem.getPose().getTranslation().getDistance(
                                RobotConstants.FieldConstants.hub.get().getTranslation());
                            boolean distOk = Math.abs(hubDist - midRange) <= posTol;
                            boolean angleOk = driveSubsystem.isAimedAtHub(Math.toRadians(3.0));
                            allowedByDrive = distOk && angleOk;
                        }
                        if (driveSubsystem.isAimModeActive()) {
                            allowedByAim = driveSubsystem.isAimedAtHub(Math.toRadians(6.0));
                        }
                    }

                    if ( ((shooterState == ShooterState.On && shooterUpToSpeed) && allowedByDrive && allowedByAim) || intakeState == IntakeState.REV ) {
                        indexerState = IndexerState.SHOOTER;
                        indexerController.setVoltage(edu.wpi.first.units.Units.Volts.of(-11));
                    } else if ( intakeState == IntakeState.ON ) {
                        indexerState = IndexerState.HOPPER;
                        indexerController.setVoltage(edu.wpi.first.units.Units.Volts.of(11));
                    } else {
                        indexerState = IndexerState.OFF;
                        indexerController.setVoltage(edu.wpi.first.units.Units.Volts.of(0));
                    }
                });
    }

    public IndexerState getState() {
        return indexerState;
    }    

    /**
     * Updates telemetry data for the indexer motor controller.
     * Called periodically by the command scheduler.
     */
    @Override
    public void periodic() {
        logSelf();

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