package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.spi.CurrencyNameProvider;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterState;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import frc.robot.Subsystems.Drive;
import frc.robot.Libs.FuelSim;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Meters;
// Radians import no longer used because turret is fixed at 180 degrees
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
    
    /** Configuration for the smart motor controller including PID, feedforward, and gearing. */
    public SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants) - no motion profiling for shooter
        .withClosedLoopController(
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KP, 
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KI, 
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KD)
        .withSimClosedLoopController(
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KP, 
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KI, 
            RobotConstants.ShooterSubsystemConstants.SHOOTER_KD)
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("ShooterMotor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        .withGearing(1)
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

    /** Reference to the fuel simulation manager (optional - null on robot/hardware). */
    private final FuelSim fuelSim;

    /** Current state of the shooter mechanism. */
    private ShooterState current_state = ShooterState.Off;
    /** Target state of the shooter mechanism. */
    private ShooterState state = ShooterState.Off;
    // Simulation spawn timing
    private long lastSpawnNs = 0;
    private final double shotsPerSecond = 3.0; // configurable rate for sim
    // Last computed ideal intercept (published continuously)
    private volatile double lastIdealX = Double.NaN;
    private volatile double lastIdealY = Double.NaN;
    private volatile double lastIdealT = Double.NaN;

    /**
     * Update the last computed ideal intercept pose/time for visualization.
     * Called by external code (for example RobotContainer) when computing lead shots.
     */
    public void setLastIdealPose(double x, double y, double t) {
        this.lastIdealX = x;
        this.lastIdealY = y;
        this.lastIdealT = t;
    }

    /**
     * Constructs a new Shooter subsystem.
     * Initializes motor controllers, flywheel, SysId routine, and sets up default command.
     */
    public Shooter(FuelSim fuelSim) {
        this.fuelSim = fuelSim;
        // Configure the follower motor using SparkFlexConfig to follow the leader motor inverted
        
        // Initialize SysId routine with 7V max voltage and data logging
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                edu.wpi.first.units.Units.Volts.of(1).per(edu.wpi.first.units.Units.Second), // Ramp rate: 1V per second
                edu.wpi.first.units.Units.Volts.of(7), // Max voltage: 7V
                null, // Default timeout (no timeout)
                null  // Default log state
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> smc.setVoltage(volts),
                log -> {
                    // Log motor data for SysId analysis
                    log.motor("shooter")
                        .voltage(edu.wpi.first.units.Units.Volts.mutable(
                            shooter_motor_controller.getAppliedOutput() * shooter_motor_controller.getBusVoltage()))
                        .angularPosition(edu.wpi.first.units.Units.Rotations.mutable(
                            shooter_motor_controller.getEncoder().getPosition()))
                        .angularVelocity(edu.wpi.first.units.Units.RotationsPerSecond.mutable(
                            shooter_motor_controller.getEncoder().getVelocity() / 60.0));
                },
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
        
        // setDefaultCommand(shooterHandler());
    }

    /**
     * Launch a fuel using FuelSim while leading the target. Returns true if a launch was performed.
     */
    public boolean leadAndLaunch(Pose2d robotPose, Pose2d targetPose, Translation2d targetVelocity) {
        if (this.fuelSim == null) return false;

        try {
            // No turret on robot and fixed elevation: always launch with 80 degrees elevation
            fuelSim.launchFuel(MetersPerSecond.of(7), Degrees.of(80.0), Degrees.of(180.0), Meters.of(0.9));
            return true;
        } catch (IllegalStateException ex) {
            return false;
        }
    }

    /**
     * Returns the last computed ideal intercept pose if available.
     * The pose is in field coordinates and has a zero rotation.
     */
    public Optional<Pose2d> getLastIdealPose() {
        if (Double.isNaN(lastIdealX) || Double.isNaN(lastIdealY)) return Optional.empty();
        return Optional.of(new Pose2d(lastIdealX, lastIdealY, new edu.wpi.first.math.geometry.Rotation2d(0.0)));
    }

    /**
     * Gets the SysId quasistatic forward routine command.
     * 
     * @return command that runs the SysId quasistatic forward test
     */
    public Command getSysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    /**
     * Gets the SysId dynamic forward routine command.
     * 
     * @return command that runs the SysId dynamic forward test
     */
    public Command getSysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    /**
     * Gets the complete SysId routine that runs all four tests in sequence.
     * Runs: Quasistatic Forward → Quasistatic Reverse → Dynamic Forward → Dynamic Reverse
     * 
     * @return command that runs the complete SysId characterization routine
     */
    public Command getCompleteSysIdRoutine() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Gets the current state of the shooter.
     * 
     * @return the current shooter state
     */
    public ShooterState getState() { return state; }

    /**
     * Checks if the shooter is up to speed.
     * Compares current velocity to target velocity with a tolerance.
     * 
     * @return true if shooter is at or above target speed (within 5% tolerance), false otherwise
     */
    public boolean isUpToSpeed() {
        if (state == ShooterState.Off) {
            return false;
        }
        
        // Get current velocity in RPM from the motor controller encoder
        double currentVelocityRPM = shooter_motor_controller.getEncoder().getVelocity();
        
        // Get target velocity based on current state
        double targetVelocityRPM;
        if (state == ShooterState.On) {
            targetVelocityRPM = 50;
        } else if (state == ShooterState.Backwards) {
            targetVelocityRPM = -50;
        } else {
            return false;
        }
        
        // Calculate tolerance (5% of target speed)
        double tolerance = Math.abs(targetVelocityRPM * 0.10);
        
        // Check if within tolerance
        boolean atSpeed = currentVelocityRPM >= 50;
        
        // Log to SmartDashboard
        SmartDashboard.putBoolean("Shooter/IsUpToSpeed", atSpeed);
        SmartDashboard.putNumber("Shooter/CurrentVelocityRPM", currentVelocityRPM);
        SmartDashboard.putNumber("Shooter/TargetVelocityRPM", targetVelocityRPM);
        SmartDashboard.putNumber("Shooter/VelocityError", Math.abs(currentVelocityRPM - targetVelocityRPM));

        return currentVelocityRPM >= 50;

        // return atSpeed;
    }

    /**
     * Creates a command to start the shooter at full speed.
     * 
     * @return command that runs shooter at full forward speed
     */
    public CCommand Shoot() {
        return cCommand("StartShooting")
                .onInitialize(() -> {
                    state = ShooterState.On;
                    shooter_controller.setMechanismVelocitySetpoint(ShooterSubsystemConstants.forwardsOnSpeeds);
                    // In simulation, spawn a projectile when shooting starts so visuals match the command
                    if (RobotBase.isSimulation()) {
                        // approximate muzzle speed = (RPM / 60) * circumference
                        // spawn timer initialized; actual muzzle speed computed when needed
                        // elevation 0 rad (flat shot). Adjust if you want lofting.
                        // prepare continuous spawn timer
                        lastSpawnNs = System.nanoTime();
                    }
                })
                .onEnd(() -> {
                    state = ShooterState.Off;
                    shooter_motor_controller.setVoltage(0);
                    lastSpawnNs = 0;
                });
    }

    /**
     * Creates a command to run the shooter backwards.
     * 
     * @return command that runs shooter backwards
     */
    public CCommand ShootBackwards() {
        return cCommand("ShootBackwards")
                .onInitialize(() -> {
                    state = ShooterState.Backwards;
                    shooter_controller.setMechanismVelocitySetpoint(ShooterSubsystemConstants.backwardsOnSpeeds);
                })
                .onEnd(() -> {
                    state = ShooterState.Off;
                    shooter_motor_controller.setVoltage(0);
                });
    }

    /**
     * Creates a command to stop the shooter.
     * 
     * @return command that stops the shooter
     */
    public CCommand StopShooting() {
        return cCommand("StopShooting")
                .onExecute(() -> {
                    state = ShooterState.Off;
                    shooter_motor_controller.setVoltage(0);
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
        
        // Log whether shooter is up to speed
        isUpToSpeed();
    }

    /**
     * Iterates the shooter controller simulation.
     * Called periodically during simulation mode.
     */
    @Override
    public void simulationPeriodic() {
        shooter_controller.simIterate();
        // Step the projectile sim at 20ms
        // Spawn simulated fuel when running the shooter so the FuelSim has projectiles to simulate
        if (this.fuelSim != null && state == ShooterState.On) {
            long now = System.nanoTime();
            double intervalNs = 1e9 / shotsPerSecond;
            if (lastSpawnNs == 0) lastSpawnNs = now;
            if (now - lastSpawnNs >= intervalNs) {
                lastSpawnNs = now;
                // Use the shooter elevation (~70 degrees) and zero turret yaw (robot-relative)
                try {
                    fuelSim.launchFuel(MetersPerSecond.of(7), Degrees.of(80.0), Degrees.of(180.0), Meters.of(0.9));
                } catch (IllegalStateException ex) {
                    // Robot not registered with fuelSim yet; ignore spawn attempt
                }
            }
        }
        // Publish last computed ideal target each simulation cycle for visualization
        var tbl = NetworkTableInstance.getDefault().getTable("Shooter");
        tbl.getEntry("IdealTargetPose").setDoubleArray(new double[] { lastIdealX, lastIdealY, 0.0 });
        tbl.getEntry("IdealTargetTime").setDouble(lastIdealT);
    }
}
