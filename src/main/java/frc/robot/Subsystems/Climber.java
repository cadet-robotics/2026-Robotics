package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Dashboard;
import frc.robot.Constants.RobotConstants;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import yams.gearing.MechanismGearing;
import yams.gearing.GearBox;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class Climber extends CSubsystem {
    public SparkMax climber_motor_controller = new SparkMax(25, MotorType.kBrushless);
    
    /** Limit switch on DIO 0 - triggers when climber is at bottom position */
    private final DigitalInput limitSwitch = new DigitalInput(0);

    private final double zero_position = RobotConstants.ClimberSubsystemConstants.ZERO_POSITION;
    private final double climb_position = RobotConstants.ClimberSubsystemConstants.CLIMB_POSITION;
    private final double max_position = RobotConstants.ClimberSubsystemConstants.MAX_POSITION;

    private boolean zeroed = false;

    public RelativeEncoder climber_encoder = climber_motor_controller.getEncoder();

    public Climber() {
        this.setName("ClimberSubsystem");
    }

    public SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig()
        .withSubsystem(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        .withClosedLoopController(1,0,0)
        // .withClosedLoopController(10,0,0.3)
        .withSimClosedLoopController( 10, 0, 0.3 )
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withTelemetry("Climber Motor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(9)))
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withSoftLimit(Rotation.of(0), Rotation.of(18.5))
        .withStatorCurrentLimit(Amps.of(40));

    public SmartMotorController climber_motor = new SparkWrapper(
        climber_motor_controller,
        DCMotor.getNEO(1),
        smc_config
    );
    
    public ConditionalCommand climbUp() {
        return cCommand("ClimberSubsystem.ClimbUp")
            .onInitialize(() -> {
                climber_motor.startClosedLoopController();
            })
            .onExecute(() -> {
                // Use position control to move to max position
                this.climber_motor.setPosition(Rotation.of(max_position));
            })
            .isFinished(() -> {
                // Command finishes when position is reached (within tolerance)
                return isAtPosition(max_position);
            }).onlyIf(()->zeroed);
    }

    public ConditionalCommand climb() {
        return cCommand("ClimberSubsystem.ClimbDown")
            .onInitialize(() -> {
                climber_motor.startClosedLoopController();
            })
            .onExecute(() -> {
                // Use position control to move to zero position
                this.climber_motor.setPosition(Rotation.of(climb_position));
            })
            .isFinished(() -> {
                // Command finishes when position is reached (within tolerance) or limit switch hit
                return isAtPosition(climb_position) || isLimitSwitchPressed();
            }).onlyIf(() -> zeroed); // Only allow climbing if we've been zeroed (to prevent trying to climb up when we don't know where we are)
    }

    public CCommand climbZero() {
        return cCommand("ClimberSubsystem.ClimbDown")
            .onInitialize(() -> {
                climber_motor.startClosedLoopController();
            })
            .onExecute(() -> {
                // Use position control to move to zero position
                this.climber_motor.setPosition(Rotation.of(zero_position));
            })
            .isFinished(
                // Command finishes when position is reached (within tolerance) or limit switch hit
                isAtPosition(zero_position) || isLimitSwitchPressed());
    }

    /**
     * Manual duty cycle control - apply 1% duty cycle to climb up
     * 
     * @return command that applies 1% duty cycle while held
     */
    public CCommand manualClimbUpVoltage() {
        return cCommand("ClimberSubsystem.ManualUpDutyCycle")
            .onInitialize(() -> {
                climber_motor.stopClosedLoopController();
            })
            .onExecute(() -> {
                this.climber_motor.setDutyCycle(0.5);
            })
            .onEnd(() -> {
                System.out.println("I've Ended");
                this.climber_motor.setDutyCycle(0);
            });
    }

    /**
     * Manual duty cycle control - apply -1% duty cycle to climb down
     * 
     * @return command that applies -1% duty cycle while held
     */
    public CCommand manualClimbDownVoltage() {
        return cCommand("ClimberSubsystem.ManualDownDutyCycle")
            .onInitialize(() -> {
                climber_motor.stopClosedLoopController();
            })
            .onExecute(() -> {
                climber_motor.setDutyCycle(-0.5);
            })
            .onEnd(() -> {
                this.climber_motor.setDutyCycle(0);
            });
    }

    private boolean isAtPosition(double target_position) {
        double currentPos = this.climber_encoder.getPosition();
        return Math.abs(currentPos - target_position) < 0.02; // 5% tolerance
    }

    /**
     * Gets the state of the limit switch.
     * 
     * @return true if limit switch is pressed (climber at bottom), false otherwise
     */
    public boolean isLimitSwitchPressed() {
        return !limitSwitch.get(); // Limit switches are typically normally-open, so invert
    }

    @Override
    public void periodic() {
        logSelf();

        // Update telemetry
        climber_motor.updateTelemetry();
        
        Dashboard.setElevatorStatus(isAtPosition(zero_position));
        
        // Reset encoder when limit switch is pressed (auto-zero)
        if (!zeroed && isLimitSwitchPressed()) {
            climber_encoder.setPosition(zero_position);
            zeroed = true;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update simulation
        climber_motor.simIterate();
    }
}
