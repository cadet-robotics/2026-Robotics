package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterState;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Shooter Subsystem with SysId Characterization Support
 * 
 * Use the SysId routine commands to characterize the shooter mechanism
 * and generate feedforward/feedback constants.
 */
public class Shooter extends CSubsystem {
    /** Motor controller for the shooter mechanism. */
    public SparkFlex shooter_motor_controller = new SparkFlex(10, SparkLowLevel.MotorType.kBrushless);
    public SparkFlex shooter_follower_motor = new SparkFlex(11, SparkLowLevel.MotorType.kBrushless);
    
    /** Configuration for the smart motor controller including PID, feedforward, and gearing. */
    public SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KP, 
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KI, 
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KD, 
            DegreesPerSecond.of(90), 
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KP, 
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KI, 
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KD, 
            DegreesPerSecond.of(90), 
            DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("ShooterMotor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
        // You could also use .withGearing(12) which does the same thing.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    /** Smart motor controller wrapper for the shooter motor. */
    public SmartMotorController smc = new SparkWrapper(shooter_motor_controller, DCMotor.getNeoVortex(1), smc_config);
    
    /** Configuration for the flywheel mechanism including diameter and mass. */
    public FlyWheelConfig shooter_config = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))  // Example: 4 inch diameter flywheel
        .withMass(Pounds.of(1));      // Example: 1 pound flywheel
    /** Flywheel controller for managing shooter wheel velocity. */
    public FlyWheel shooter_controller = new FlyWheel(shooter_config);
    
    /** SysId routine for motor characterization. */
    private final SysIdRoutine sysIdRoutine;

    public static enum ShooterState {
        On,
        Rev, // Unused but may come up if jamming happens
        Off,
        Backwards,
        ManualForward,
        ManualBackward,
    }

    /** Current state of the shooter mechanism. */
    private ShooterState current_state = ShooterState.OFF;
    /** Target state of the shooter mechanism. */
    private ShooterState state = ShooterState.OFF;

    /**
     * Constructs a new Shooter subsystem.
     * Initializes motor controllers, flywheel, SysId routine, and sets up default command.
     */
    public Shooter() {
        // Configure the follower motor using SparkFlexConfig to follow the leader motor inverted
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.follow(this.shooter_motor_controller).inverted(true);
        this.shooter_follower_motor.configure(followerConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        
        // Initialize SysId routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> smc.setVoltage(volts),
                null, // No log consumer (can add if needed)
                this
            )
        );
        
        // Register SysId commands with SmartDashboard
        SmartDashboard.putData("Shooter/SysId Quasistatic Forward", 
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Shooter/SysId Quasistatic Reverse", 
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Shooter/SysId Dynamic Forward", 
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Shooter/SysId Dynamic Reverse", 
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        
        setDefaultCommand(shooterHandler());
    }

    /**
     * Gets the current state of the shooter.
     * 
     * @return the current shooter state
     */
    public ShooterState getState() { return state; }

    /**
     * Creates a command to start the shooter at full speed.
     * 
     * @return command that sets shooter state to On
     */
    public CCommand Shoot() {
        return cCommand("StartShooting")
                .onInitialize(() -> {
                    state = ShooterState.ON;
                });
    }

    public CCommand ShootBackwards() {
        return cCommand("ShootBackwards")
                .onInitialize(() -> {
                    this.state = Shooter.ShooterState.Backwards;
                });
    }

    public CCommand ManualSpinForward() {
        return cCommand("ManualSpinForward")
                .onInitialize(() -> {
                    this.state = Shooter.ShooterState.ManualForward;
                });
    }

    public CCommand ShootBackwards() {
        return cCommand("ShootBackwards")
                .onInitialize(() -> {
                    this.state = Shooter.ShooterState.Backwards;
                });
    }

    public CCommand ManualSpinForward() {
        return cCommand("ManualSpinForward")
                .onInitialize(() -> {
                    this.state = Shooter.ShooterState.ManualForward;
                });
    }

    public CCommand ManualSpinBackward() {
        return cCommand("ManualSpinBackward")
                .onInitialize(() -> {
                    this.state = Shooter.ShooterState.ManualBackward;
                });
    }

    public CCommand ManualSpinBackward() {
        return cCommand("ManualSpinBackward")
                .onInitialize(() -> {
                    this.state = Shooter.ShooterState.ManualBackward;
                });
    }

    /**
     * Creates a command to stop the shooter.
     * 
     * @return command that sets shooter state to Off
     */
    public CCommand StopShooting() {
        return cCommand("StopShooting")
                .onInitialize(() -> {
                    state = ShooterState.OFF;
                });
    }

    /**
     * Creates the default command that handles shooter state transitions.
     * Monitors state changes and updates flywheel velocity accordingly.
     * 
     * @return command that handles automatic shooter control
     */
    public CCommand shooterHandler() {
        return cCommand()
            .onExecute(() -> {
                if ( state != current_state ) {
                    switch (state) {
                        case OFF:
                            current_state = ShooterState.OFF;
                            shooter_controller.setSpeed(RPM.of(0));
                            break;
                        case ON:
                            current_state = ShooterState.ON;
                            shooter_controller.setSpeed(RobotConstants.ShooterSubsystemConstants.forwardsOnSpeeds);
                            break;
                        case Backwards:
                            this.current_state = Shooter.ShooterState.Backwards;
                            this.shooter_controller.setSpeed(Constants.ShooterSubsystemConstants.backwardsOnSpeeds);
                            break;
                        case ManualForward:
                            this.current_state = Shooter.ShooterState.ManualForward;
                            this.shooter_controller.setSpeed(Constants.ShooterSubsystemConstants.manualSpinSpeed);
                            break;
                        case ManualBackward:
                            this.current_state = Shooter.ShooterState.ManualBackward;
                            this.shooter_controller.setSpeed(RPM.of(-Constants.ShooterSubsystemConstants.manualSpinSpeed.in(RPM)));
                            break;
                        case REV:
                            current_state = ShooterState.REV;
                            // Rev state - could be used for different speed (half of forward speed)
                            shooter_controller.setSpeed(RPM.of(RobotConstants.ShooterSubsystemConstants.forwardsOnSpeeds.in(RPM) / 2));
                            break;
                    }
                }
            });
    }

    /**
     * Updates telemetry data for the shooter controller.
     * Called periodically by the command scheduler.
     */
    @Override
    public void periodic() {
        // Update telemetry
        shooter_controller.updateTelemetry();
    }

    /**
     * Iterates the shooter controller simulation.
     * Called periodically during simulation mode.
     */
    @Override
    public void simulationPeriodic() {
        shooter_controller.simIterate();
    }
}
