package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.Constants.IndexerSubsystemConstants;

import static edu.wpi.first.units.Units.*;

public class Indexer extends CSubsystem {
    private final SparkFlex indexer_motor_controller = new SparkFlex(1, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
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

    private final SmartMotorController indexer_controller = new SparkWrapper(
            this.indexer_motor_controller,
            DCMotor.getNeoVortex(1),
            this.smc_config
    );

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    private Shooter shooter_subsystem;
    private Intake intake_subsystem;

    private static enum IndexerState {
        Shooter,
        Hopper,
        Off
    }
    private IndexerState state = IndexerState.Off;

    public Indexer( Shooter shooter_subsystem, Intake intake_subsystem ) {
        this.shooter_subsystem = shooter_subsystem;
        this.intake_subsystem = intake_subsystem;
        
        // Initialize SysId routine
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> this.indexer_controller.setVoltage(volts),
                null, // No log consumer (can add if needed)
                this
            )
        );
        
        // Register SysId commands with SmartDashboard
        SmartDashboard.putData("Indexer/SysId Quasistatic Forward", 
            this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Indexer/SysId Quasistatic Reverse", 
            this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Indexer/SysId Dynamic Forward", 
            this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Indexer/SysId Dynamic Reverse", 
            this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        
        this.setDefaultCommand(indexerHandler());
    }

    public CCommand shooterIndexing() {
        return cCommand( "ShootingIndexing")
                .onInitialize(() -> state = Indexer.IndexerState.Shooter);
    }

    public CCommand hopperIndexing() {
        return cCommand( "HopperIndexing")
                .onInitialize(() -> state = Indexer.IndexerState.Hopper);
    }

    public CCommand stopIndexer() {
        return cCommand( "StopIndexer")
                .onInitialize(() -> state = Indexer.IndexerState.Off);
    }

    public CCommand indexerHandler() {
        return cCommand("IndexerHandler")
                .onExecute(() -> {
                   if ( this.shooter_subsystem.getState() == Shooter.ShooterState.On ) {
                       this.state = Indexer.IndexerState.Shooter;
                       this.indexer_controller.setVelocity(IndexerSubsystemConstants.forwardsOnSpeeds);
                   } else if ( this.intake_subsystem.getState() == Intake.IntakeState.On ) {
                       this.state = Indexer.IndexerState.Hopper;
                       this.indexer_controller.setVelocity(IndexerSubsystemConstants.backwardsOnSpeeds);
                   } else {
                       this.state = Indexer.IndexerState.Off;
                       this.indexer_controller.setVelocity(RPM.of(0));
                   }
                });
    }

    @Override
    public void periodic() {
        this.indexer_controller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        this.indexer_controller.simIterate();
    }
}