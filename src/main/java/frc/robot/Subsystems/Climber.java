package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotConstants;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.gearing.GearBox;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class Climber extends CSubsystem {
    public SparkMax climber_motor_controller = new SparkMax(25, MotorType.kBrushless);
    
    /** Limit switch on DIO 0 - triggers when climber is at bottom position */
    private final DigitalInput limitSwitch = new DigitalInput(0);

    private final float zero_position = RobotConstants.ClimberSubsystemConstants.ZERO_POSITION;
    private final float climb_position = RobotConstants.ClimberSubsystemConstants.CLIMB_POSITION;
    private final float max_position = RobotConstants.ClimberSubsystemConstants.MAX_POSITION;

    public RelativeEncoder climber_encoder = climber_motor_controller.getEncoder();

    public Climber() {}

    public SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig()
        .withSubsystem(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        .withClosedLoopController( new ExponentialProfilePIDController(1, 0, 0, ExponentialProfilePIDController.createConstraints(Volts.of(12), DegreesPerSecond.of(5600 * 360), DegreesPerSecondPerSecond.of(5600 * 360))))
        .withSimClosedLoopController(1, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withTelemetry("Climber Motor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(9)))
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40));

    public SmartMotorController climber_motor = new SparkWrapper(
        climber_motor_controller,
        DCMotor.getNEO(1),
        smc_config
    );
    
    public CCommand climbUp() {
        return cCommand("ClimberSubsystem.ClimbUp")
            .onExecute(() -> {
                // Use position control to move to max position
                this.climber_motor.setPosition(Degree.of(max_position));
            })
            .isFinished(() -> {
                // Command finishes when position is reached (within tolerance)
                return isAtPosition(max_position);
            });
    }

    public CCommand climb() {
        return cCommand("ClimberSubsystem.ClimbDown")
            .onExecute(() -> {
                // Use position control to move to zero position
                this.climber_motor.setPosition(Degree.of(climb_position));
            })
            .isFinished(() -> {
                // Command finishes when position is reached (within tolerance) or limit switch hit
                return isAtPosition(climb_position) || isLimitSwitchPressed();
            });
    }

    public CCommand climbZero() {
        return cCommand("ClimberSubsystem.ClimbDown")
            .onExecute(() -> {
                // Use position control to move to zero position
                this.climber_motor.setPosition(Degree.of(zero_position));
            })
            .isFinished(() -> {
                // Command finishes when position is reached (within tolerance) or limit switch hit
                return isAtPosition(zero_position) || isLimitSwitchPressed();
            });
    }

    /**
     * Manual duty cycle control - apply 1% duty cycle to climb up
     * 
     * @return command that applies 1% duty cycle while held
     */
    public CCommand manualClimbUpVoltage() {
        return cCommand("ClimberSubsystem.ManualUpDutyCycle")
            .onExecute(() -> {
                this.climber_motor_controller.setVoltage(Volts.of(2));
            })
            .onEnd(() -> {
                this.climber_motor_controller.setVoltage(Volts.of(0));
            });
    }

    /**
     * Manual duty cycle control - apply -1% duty cycle to climb down
     * 
     * @return command that applies -1% duty cycle while held
     */
    public CCommand manualClimbDownVoltage() {
        return cCommand("ClimberSubsystem.ManualDownDutyCycle")
            .onExecute(() -> {
                this.climber_motor_controller.setVoltage(Volts.of(-2));
            })
            .onEnd(() -> {
                this.climber_motor_controller.setVoltage(Volts.of(0));
            });
    }

    private boolean isAtPosition(float target_position) {
        double currentPos = this.climber_encoder.getPosition();
        return Math.abs(currentPos - target_position) < 0.05; // 5% tolerance
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
        // Update telemetry
        climber_motor.updateTelemetry();
        
        // Publish limit switch state to SmartDashboard
        SmartDashboard.putBoolean("Climber/Limit Switch", isLimitSwitchPressed());
        
        // Publish alternate encoder readings to SmartDashboard
        SmartDashboard.putNumber("Climber/Alternate Encoder Position", climber_encoder.getPosition());
        SmartDashboard.putNumber("Climber/Alternate Encoder Velocity", climber_encoder.getVelocity());
        
        // Reset encoder when limit switch is pressed (auto-zero)
        if (isLimitSwitchPressed()) {
            climber_encoder.setPosition(zero_position);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update simulation
        climber_motor.simIterate();
    }
}
