package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import yams.gearing.MechanismGearing;
import yams.gearing.GearBox;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class Climber extends CSubsystem {
    public SparkMax climber_motor_controller = new SparkMax(25, MotorType.kBrushless);
    public RelativeEncoder climber_encoder = climber_motor_controller.getAlternateEncoder();

    public final float zero_position = 0;
    public final float max_position = 1; // Temp Value, Tune on Robot;

    public SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig()
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.001, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withTelemetry("IntakeMotor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 4)))
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40));

    public SmartMotorController climber_motor = new SparkWrapper(
        climber_motor_controller,
        DCMotor.getNEO(1),
        smc_config
    );
    
    public CCommand climbUp() {
        return cCommand("ClimerSubsystem.ClimbUp")
            .onInitialize(() -> {
                this.climber_motor.setVoltage(Volts.of(2));
            })
            .onEnd(() -> {
                this.climber_motor.setVoltage(Volts.of(0));
            });
    }

    public CCommand climbDown() {
        return cCommand("ClimerSubsystem.ClimbDown")
            .onInitialize(() -> {
                this.climber_motor.setVoltage(Volts.of(-2));
            })
            .onEnd(() -> {
                this.climber_motor.setVoltage(Volts.of(0));
            });
    }
}
