package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeState;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Intake subsystem that controls the mechanism for collecting game pieces.
 * Manages motor control for intake operations including on, off, and reverse states.
 */
public class Intake extends CSubsystem {
    /** Motor controller for the intake mechanism. */
    private final SparkMax intakeMotorController = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
    /** Configuration for the smart motor controller including PID, feedforward, and gearing. */
    private final SmartMotorControllerConfig smcConfig  = new SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("IntakeMotor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
        // You could also use .withGearing(12) which does the same thing.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    /** Smart motor controller wrapper for the intake motor. */
    private final SmartMotorController intakeController = new SparkWrapper( intakeMotorController, DCMotor.getNeoVortex(1), smcConfig);

    /** SysId routine for motor characterization. */
    private final SysIdRoutine sysIdRoutine;

    /** Current state of the intake mechanism. */
    private IntakeState currentState = IntakeState.OFF;
    /** Target state of the intake mechanism. */
    private IntakeState state = IntakeState.OFF;

    /**
     * Gets the current state of the intake.
     * 
     * @return the current intake state
     */
    public IntakeState getState() { return state; }

    /**
     * Constructs a new Intake subsystem.
     * Initializes motor controller, SysId routine, and sets up default command.
     */
    public Intake() {
        // Initialize SysId routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> intakeController.setVoltage(volts),
                null, // No log consumer (can add if needed)
                this
            )
        );
        
        // Register SysId commands with SmartDashboard
        SmartDashboard.putData("Intake/SysId Quasistatic Forward", 
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Intake/SysId Quasistatic Reverse", 
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Intake/SysId Dynamic Forward", 
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Intake/SysId Dynamic Reverse", 
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        
        // setDefaultCommand(intakeHandler());
    }

    /**
     * Creates a command to turn the intake on.
     * 
     * @return command that sets intake state to On
     */
    public CCommand SetIntakeOn() {
        return cCommand().onInitialize(() -> {
            // state = IntakeState.ON;
            this.intakeController.setVoltage(Volts.of(3));
        });
    }

    /**
     * Creates a command to turn the intake off.
     * 
     * @return command that sets intake state to Off
     */
    public CCommand SetIntakeOff() {
        return cCommand().onInitialize(() -> {
            // state = IntakeState.OFF;
            this.intakeController.setVoltage(Volts.of(0));
        });
    }

    /**
     * Creates the default command that handles intake state transitions.
     * Monitors state changes and updates motor velocity accordingly.
     * 
     * @return command that handles automatic intake control
     */
    public CCommand intakeHandler() {
       return cCommand("IntakeHandler")
           .onExecute(() -> {
               if ( state != currentState ) {
                   switch (state) {
                       case OFF:
                            currentState = IntakeState.OFF;
                            intakeController.setVelocity(RPM.of(0));
                            break;
                       case ON:
                           currentState = IntakeState.ON;
                           intakeController.setVelocity(RPM.of(100));
                           break;
                       case REV:
                           currentState = IntakeState.REV;
                           // TODO: Implement reverse/barfing speed
                           intakeController.setVelocity(RPM.of(-100));
                           break;
                   }
               }
           });
    }

    /**
     * Updates telemetry data for the intake motor controller.
     * Called periodically by the command scheduler.
     */
    @Override
    public void periodic() {
        intakeController.updateTelemetry();
    }

    /**
     * Iterates the motor controller simulation.
     * Called periodically during simulation mode.
     */
    @Override
    public void simulationPeriodic() {
        intakeController.simIterate();
    }
}