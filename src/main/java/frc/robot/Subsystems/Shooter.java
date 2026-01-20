package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Libs.APIDTunerSMCWrapper;
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

import static edu.wpi.first.units.Units.*;

/**
 * Shooter Subsystem with PID AutoTuning Support
 * 
 * === AUTOTUNING USAGE GUIDE ===
 * 
 * 1. PREPARATION:
 *    - Ensure robot is secured (wheels off ground or on blocks)
 *    - Clear area around shooter mechanism
 *    - Have emergency stop ready
 * 
 * 2. START AUTOTUNING:
 *    - Open SmartDashboard or Shuffleboard
 *    - Find "Shooter/AutoTune/Enable" boolean
 *    - Set it to TRUE to start autotuning
 *    - Watch "Shooter/Status" for progress
 *    - The shooter will oscillate for 30-60 seconds
 * 
 * 3. DURING AUTOTUNING:
 *    - DO NOT send shooter commands
 *    - DO NOT interfere with mechanism
 *    - Watch for "Shooter/AutoTune/Running" to turn FALSE when complete
 *    - Check SmartDashboard for calculated PID values:
 *      * Shooter/kp
 *      * Shooter/ki  
 *      * Shooter/kd
 * 
 * 4. SAVE PID VALUES:
 *    - If values look reasonable, set "Shooter/AutoTune/SavePID" to TRUE
 *    - Values are saved to robot persistent storage
 *    - Restart robot to apply saved values
 * 
 * 5. RESET TO DEFAULTS (if needed):
 *    - Set "Shooter/AutoTune/ResetToDefaults" to TRUE
 *    - Restart robot to apply
 * 
 * === SIMULATION TESTING ===
 * - Autotuning works with YAMS simulation
 * - Run robot simulation and follow same steps
 * - Simulated physics will create realistic oscillations
 * 
 * === TROUBLESHOOTING ===
 * - If tuning never completes: Increase lookback time or output step
 * - If oscillations are too large: Decrease output step (default 5V)
 * - If no oscillation occurs: Increase output step or decrease noise band
 * - Check console for "Saved Shooter PID" message after saving
 */
public class Shooter extends CSubsystem {
    public SparkFlex shooter_motor_controller = new SparkFlex(3, SparkLowLevel.MotorType.kBrushless);
    public SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants) - Load from Constants (which loads from persistent storage)
        .withClosedLoopController(
            Constants.ShooterSubsystemConstants.getShooterKp(), 
            Constants.ShooterSubsystemConstants.getShooterKi(), 
            Constants.ShooterSubsystemConstants.getShooterKd(), 
            DegreesPerSecond.of(90), 
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(
            Constants.ShooterSubsystemConstants.getShooterKp(), 
            Constants.ShooterSubsystemConstants.getShooterKi(), 
            Constants.ShooterSubsystemConstants.getShooterKd(), 
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
    
    // PID AutoTuner - initialized in constructor
    private APIDTunerSMCWrapper autotuner;
    private CCommand autoTuneCommand;  // Cached command instance

    public static enum ShooterState {
        On,
        Rev, // Unused but may come up if jamming happens
        Off,
    }
    private ShooterState current_state = Shooter.ShooterState.Off;
    private ShooterState state = Shooter.ShooterState.Off;

    public Shooter() {
        // Initialize autotuner with SAFE output step (5V instead of dangerous 50V default)
        this.autotuner = new APIDTunerSMCWrapper("Shooter", this.smc);
        this.autotuner.setOutputStep(5.0);  // SAFE: 5 volts for 12V battery system
        this.autotuner.setNoiseBand(0.5);   // Adjust based on mechanism noise
        this.autotuner.setLookbackSec(15);  // 15 second lookback window
        
        // Create the autotune command once
        this.autoTuneCommand = this.createAutoTuneCommand();
        
        // Register the PID AutoTune command with SmartDashboard
        SmartDashboard.putData("Mechanisms/Commands/Shooter/PID AutoTune", this.autoTuneCommand);
        
        this.setDefaultCommand(shooterHandler());
    }

    public ShooterState getState() { return this.state; }

    /**
     * Command to run PID autotuning on the shooter flywheel.
     * Uses relay feedback method - motor alternates between On/Off speeds to identify system dynamics.
     * WARNING: This will oscillate the shooter motor - do not run with game pieces!
     */
    private CCommand createAutoTuneCommand() {
        return cCommand("PID AutoTune")
                .onInitialize(() -> {
                    System.out.println("[AutoTune] Command Initialize - Starting autotuning");
                    this.autotuner.changeAutoTune();  // Start autotuning
                    SmartDashboard.putString("Shooter/Status", "AUTOTUNING - Motor will oscillate!");
                    SmartDashboard.putBoolean("Shooter/AutoTune/Running", true);
                    System.out.println("[AutoTune] isTuning: " + this.autotuner.isTuning());
                })
                .onExecute(() -> {
                    // Run the autotuning algorithm (reads sensor, detects peaks, updates output signal)
                    this.autotuner.tuningLoop();
                    
                    // Read the relay state from the autotuner and apply appropriate speed
                    double autotunerSignal = this.autotuner.getCurrentOutput();
                    
                    if (autotunerSignal > 0) {
                        // High relay state - use the actual "On" speed for real-world tuning
                        this.shooter_controller.setSpeed(Constants.ShooterSubsystemConstants.forwardsOnSpeeds);
                        System.out.println("[AutoTune] HIGH - Setting speed to: " + Constants.ShooterSubsystemConstants.forwardsOnSpeeds.in(RPM));
                    } else {
                        // Low relay state - motor off
                        this.shooter_controller.setSpeed(RPM.of(0));
                        System.out.println("[AutoTune] LOW - Setting speed to: 0");
                    }
                    
                    // Update telemetry
                    double currentSpeed = this.smc.getMeasurementVelocity().in(MetersPerSecond);
                    SmartDashboard.putNumber("Shooter/AutoTune/CurrentSpeed", currentSpeed);
                    SmartDashboard.putNumber("Shooter/AutoTune/RelayOutput", autotunerSignal);
                    SmartDashboard.putBoolean("Shooter/AutoTune/IsTuning", this.autotuner.isTuning());
                    
                    System.out.println("[AutoTune] Signal: " + autotunerSignal + ", Speed: " + currentSpeed + ", Tuning: " + this.autotuner.isTuning());
                })
                .isFinished(() -> {
                    boolean finished = !this.autotuner.isTuning();
                    if (finished) {
                        System.out.println("[AutoTune] Command finishing - tuning complete");
                    }
                    return finished;
                })
                .onEnd((interrupted) -> {
                    System.out.println("[AutoTune] Command End - interrupted: " + interrupted);
                    if (this.autotuner.isTuning()) {
                        this.autotuner.changeAutoTune();  // Cancel if still running
                    }
                    this.shooter_controller.setSpeed(RPM.of(0));  // Stop motor
                    SmartDashboard.putBoolean("Shooter/AutoTune/Running", false);
                    
                    if (interrupted) {
                        SmartDashboard.putString("Shooter/Status", "Autotuning cancelled");
                    } else {
                        // Autotuning finished successfully - save results
                        Constants.ShooterSubsystemConstants.saveShooterPID(
                            this.autotuner.getKp(),
                            this.autotuner.getKi(),
                            this.autotuner.getKd()
                        );
                        SmartDashboard.putString("Shooter/Status", "Autotuning complete - PID saved!");
                        SmartDashboard.putNumber("Shooter/PID/TunedKp", this.autotuner.getKp());
                        SmartDashboard.putNumber("Shooter/PID/TunedKi", this.autotuner.getKi());
                        SmartDashboard.putNumber("Shooter/PID/TunedKd", this.autotuner.getKd());
                    }
                });
    }

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

    @Override
    public void periodic() {
        // Update SmartDashboard with current PID values from Constants
        SmartDashboard.putNumber("Shooter/PID/CurrentKp", Constants.ShooterSubsystemConstants.getShooterKp());
        SmartDashboard.putNumber("Shooter/PID/CurrentKi", Constants.ShooterSubsystemConstants.getShooterKi());
        SmartDashboard.putNumber("Shooter/PID/CurrentKd", Constants.ShooterSubsystemConstants.getShooterKd());
        
        // Update telemetry
        this.shooter_controller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        this.shooter_controller.simIterate();
    }
}
