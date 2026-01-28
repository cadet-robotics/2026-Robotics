package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkFlex;
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

import static edu.wpi.first.units.Units.*;

public class Intake extends CSubsystem {
    private final SparkFlex intake_motor_controller = new SparkFlex(2, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig smc_config  = new SmartMotorControllerConfig(this)
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
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController intake_controller = new SparkWrapper( intake_motor_controller, DCMotor.getNeoVortex(1), smc_config);

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    public static enum IntakeState {
        On,
        Rev, // Implement for barfing
        Off,
    }
    public IntakeState getState() { return this.state; }

    private IntakeState current_state = Intake.IntakeState.Off;
    private IntakeState state = Intake.IntakeState.Off;

    public Intake() {
        // Initialize SysId routine
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> this.intake_controller.setVoltage(volts),
                null, // No log consumer (can add if needed)
                this
            )
        );
        
        // Register SysId commands with SmartDashboard
        SmartDashboard.putData("Intake/SysId Quasistatic Forward", 
            this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Intake/SysId Quasistatic Reverse", 
            this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Intake/SysId Dynamic Forward", 
            this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Intake/SysId Dynamic Reverse", 
            this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        
        this.setDefaultCommand(this.intakeHandler());
    }

    public CCommand SetIntakeOn() {
        return cCommand().onInitialize(() -> {
            this.state = Intake.IntakeState.On;
        });
    }

    public CCommand SetIntakeOff() {
        return cCommand().onInitialize(() -> {
            this.state = Intake.IntakeState.Off;
        });
    }

    public CCommand intakeHandler() {
       return cCommand("IntakeHandler")
           .onExecute(() -> {
               if ( this.state != this.current_state ) {
                   switch (this.state) {
                       case Off:
                            this.current_state = Intake.IntakeState.Off;
                            this.intake_controller.setVelocity(RPM.of(0));
                            break;
                       case On:
                           this.current_state = Intake.IntakeState.On;
                           this.intake_controller.setVelocity(RPM.of(100));
                           break;
                       case Rev:
                           this.current_state = Intake.IntakeState.Rev;
                           // TODO: Implement reverse/barfing speed
                           this.intake_controller.setVelocity(RPM.of(-100));
                           break;
                   }
               }
           });
    }

    @Override
    public void periodic() {
        this.intake_controller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        this.intake_controller.simIterate();
    }
}