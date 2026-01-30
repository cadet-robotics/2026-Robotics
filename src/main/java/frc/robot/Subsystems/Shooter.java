package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import static edu.wpi.first.units.Units.*;

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
    public SparkFlex shooter_motor_controller = new SparkFlex(3, SparkLowLevel.MotorType.kBrushless);
    public SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(
            Constants.ShooterSubsystemConstants.SHOOTER_KP, 
            Constants.ShooterSubsystemConstants.SHOOTER_KI, 
            Constants.ShooterSubsystemConstants.SHOOTER_KD, 
            DegreesPerSecond.of(90), 
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(
            Constants.ShooterSubsystemConstants.SHOOTER_KP, 
            Constants.ShooterSubsystemConstants.SHOOTER_KI, 
            Constants.ShooterSubsystemConstants.SHOOTER_KD, 
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

    public SmartMotorController smc = new SparkWrapper(shooter_motor_controller, DCMotor.getNeoVortex(1), smc_config);
    public FlyWheelConfig shooter_config = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))  // Example: 4 inch diameter flywheel
        .withMass(Pounds.of(1));      // Example: 1 pound flywheel
    public FlyWheel shooter_controller = new FlyWheel(shooter_config);
    
    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    public static enum ShooterState {
        On,
        Rev, // Unused but may come up if jamming happens
        Off,
    }
    private ShooterState current_state = Shooter.ShooterState.Off;
    private ShooterState state = Shooter.ShooterState.Off;

    public Shooter() {
        // Initialize SysId routine
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> this.smc.setVoltage(volts),
                null, // No log consumer (can add if needed)
                this
            )
        );
        
        // Register SysId commands with SmartDashboard
        SmartDashboard.putData("Shooter/SysId Quasistatic Forward", 
            this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Shooter/SysId Quasistatic Reverse", 
            this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Shooter/SysId Dynamic Forward", 
            this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Shooter/SysId Dynamic Reverse", 
            this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        
        this.setDefaultCommand(shooterHandler());
    }

    public ShooterState getState() { return this.state; }

    public CCommand Shoot() {
        return cCommand("StartShooting")
                .onInitialize(() -> {
                    this.state = Shooter.ShooterState.On;
                });
    }

    public CCommand StopShooting() {
        return cCommand("StopShooting")
                .onInitialize(() -> {
                    this.state = Shooter.ShooterState.Off;
                });
    }

    public CCommand shooterHandler() {
        return cCommand()
            .onExecute(() -> {
                if ( this.state != this.current_state ) {
                    switch (this.state) {
                        case Off:
                            this.current_state = Shooter.ShooterState.Off;
                            this.shooter_controller.setSpeed(RPM.of(0));
                            break;
                        case On:
                            this.current_state = Shooter.ShooterState.On;
                            this.shooter_controller.setSpeed(Constants.ShooterSubsystemConstants.forwardsOnSpeeds);
                            break;
                        case Rev:
                            this.current_state = Shooter.ShooterState.Rev;
                            // Rev state - could be used for different speed (half of forward speed)
                            this.shooter_controller.setSpeed(RPM.of(Constants.ShooterSubsystemConstants.forwardsOnSpeeds.in(RPM) / 2));
                            break;
                    }
                }
            });
    }

    public AngularVelocity flyShooterSpeedMath()
    {
        // TODO: Setup MegaTag2 localization
        // hubDistance should be determined using limelight MegaTag2
        double hubDistance; // ft
        double flyWheelHeight = 18.5/12; // ft
        double targetHeight = 4; // ft
        double angle = 0.977384381; // in radians (56 degrees, adjust as necessary)
        double gravity = 32.174; // ft/s^2

        // Weight of the fuel
        double projectileWeight = 0.5; // lb

        // TODO: Configure when shooter is being put together
        double flyWheelRadius; // in
        double flyWheelWeight; // lb
        double shooterRadius; // in
        double shooterWeight; // lb

        // Gear ratio of the vortex motors
        double gearRatio;

        // Calculate necessary velocity transfered to Fuel
        double projectileSurfaceVelocity = Math.sqrt(((gravity) * Math.pow(hubDistance, 2)) / (2 * Math.pow(Math.cos(angle), 2)) * (hubDistance * Math.tan(angle) - (targetHeight - flyWheelHeight)));
        
        // Calculate moment of intertia
        double flyWheelMass = flyWheelWeight / gravity;
        double shooterMass = shooterWeight / gravity;

        double flyWheelMOI = 0.5 * flyWheelMass * Math.pow(flyWheelRadius, 2) * Math.pow(gearRatio, 2);
        double shooterMOI = 0.5 * shooterMass * Math.pow(shooterRadius, 2) * Math.pow(gearRatio, 2);
        double totalMOI = flyWheelMOI + shooterMOI;
        
        // Calculate the necessary angular velocity of the shooter for the robot's distance
        double speedTransferPercentage = (20 * totalMOI) / (7 * projectileWeight * Math.pow((shooterRadius / 2),2) + 40 * totalMOI);
        double shooterSurfaceSpeed = projectileSurfaceVelocity / speedTransferPercentage;
        double shooterRPM = shooterSurfaceSpeed / shooterRadius;

        AngularVelocity velocity = RPM.of(shooterRPM);
        return velocity;
    }

    @Override
    public void periodic() {
        // Update telemetry
        this.shooter_controller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        this.shooter_controller.simIterate();
    }
}
