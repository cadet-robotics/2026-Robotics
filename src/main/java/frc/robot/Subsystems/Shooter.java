package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.FieldConstants;
import frc.robot.Constants.RobotConstants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterState;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;
import frc.robot.Libs.FuelSim;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Meters;
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
    // public SparkFlex shooter_follow_controller = new SparkFlex(11, SparkLowLevel.MotorType.kBrushless);

    private InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    {
        shooterSpeedMap.put(2.8, 3250.0);
    }

    private boolean isUsingStaticSpeed = true;

    public SparkFlexConfig shooter_SM_config = new SparkFlexConfig();
    {  
        shooter_SM_config
            // .inverted(false)
            // .idleMode(IdleMode.kCoast)
            // .smartCurrentLimit(50)
            // .secondaryCurrentLimit(50)
            .closedLoop.minOutput(0)
                .p(RobotConstants.ShooterSubsystemConstants.SHOOTER_KP)
                .i(RobotConstants.ShooterSubsystemConstants.SHOOTER_KI)
                .d(RobotConstants.ShooterSubsystemConstants.SHOOTER_KD);

    }

    private AngularVelocity targetRPM = RPM.of(1000);

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
        // Feedforwards
        .withFeedforward(new SimpleMotorFeedforward(0,0.108,0))//0.108, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("ShooterMotor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        .withGearing(1)
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(50))
        .withVendorConfig(shooter_SM_config);

    /** Smart motor controller wrapper for the shooter motor. */
    public SmartMotorController smc = new SparkWrapper(shooter_motor_controller, DCMotor.getNeoVortex(1), smc_config);
    
    /** Configuration for the flywheel mechanism including diameter and mass. */
    public FlyWheelConfig shooter_config = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))  // Example: 4 inch diameter flywheel
        .withMass(Pounds.of(4));      // Example: 1 pound flywheel
    /** Flywheel controller for managing shooter wheel velocity. */
    public FlyWheel shooter_controller = new FlyWheel(shooter_config);
    
    /** Reference to the fuel simulation manager (optional - null on robot/hardware). */
    private final FuelSim fuelSim;

    private final ColorSensorV3 ballSensor = new ColorSensorV3(Port.kOnboard);
    // Timestamp of the last moment a ball was detected by the color sensor (ns)
    private volatile long lastBallSeenNs = 0;
    // Last proximity reading from the color sensor (higher => closer). Updated in periodic().
    private static volatile int lastProximity = -1;

    /**
     * Returns the most recent proximity reading from the color sensor.
     * Public static so other subsystems (e.g., Drive) can read the value without
     * needing a subsystem reference.
     *
     * @return proximity value or -1 if not yet available
     */
    public static int getLastProximity() {
        return lastProximity;
    }

    /** Optional reference to the Intake subsystem so Shooter can request ball removal from hopper. */
    private frc.robot.Subsystems.Intake intakeSubsystem = null;
    // Optional reference to the Drive subsystem to gate simulation spawns when using autodrive
    private Drive driveSubsystem = null;

    /** Target state of the shooter mechanism. */
    private ShooterState state = ShooterState.Off;
    // Simulation spawn timing
    private long lastSpawnNs = 0;
    private final double shotsPerSecond = 3.0; // configurable rate for sim

    /**
     * Constructs a new Shooter subsystem.
     * Initializes motor controllers, flywheel, SysId routine, and sets up default command.
     */
    public Shooter(FuelSim fuelSim) {
        setName("ShooterSubsystem");

        if (fuelSim != null) {
            this.fuelSim = fuelSim;
        } else {
            this.fuelSim = null;
        }
    }

    /**
     * Set the Intake subsystem reference so the Shooter can interact with it.
     * This is provided after construction (RobotContainer sets it).
     *
     * @param intake the Intake subsystem instance
     */
    public void setIntakeSubsystem(frc.robot.Subsystems.Intake intake) {
        this.intakeSubsystem = intake;
    }

    /**
     * Provide a reference to the Drive subsystem so simulated ball spawning can be gated to
     * only occur when the robot is at the autodrive target.
     */
    public void setDriveSubsystem(Drive drive) {
        this.driveSubsystem = drive;
    }

    /**
     * Request the Intake subsystem to remove a ball from the hopper.
     * Returns true if a ball was removed, false if the hopper was empty or intake not set.
     */
    public boolean removeBallFromHopper() {
        if (this.intakeSubsystem == null) {
            return false;
        }
        return this.intakeSubsystem.removeFromHopper();
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
        // TODO, rework entire system to use velocity setpoints based on the distance, I need sleep
        // if (state == ShooterState.Off) {
        //     return false;
        // }
        
        if (isUsingStaticSpeed) {
            // Get current velocity in RPM from the motor controller encoder
            double currentVelocityRPM = shooter_motor_controller.getEncoder().getVelocity();
            
            // Get target velocity based on current state
            // double targetVelocityRPM;
            if (state == ShooterState.On) {
                // targetVelocityRPM = 50;
            } else if (state == ShooterState.Backwards) {
                // targetVelocityRPM = -50;
            } else {
                return false;
            }
            
            // Calculate tolerance (5% of target speed)
            // double tolerance = Math.abs(targetVelocityRPM * 0.5);
            
            // Check if within tolerance
            // boolean atSpeed = currentVelocityRPM >= 50;

            return currentVelocityRPM >= 2800/60;
        } else {
            if (state == ShooterState.On) {
                // Get current velocity from the motor controller encoder
                double currentVelocityRPM = shooter_motor_controller.getEncoder().getVelocity();
                try { 
                    double targetVelocityRPM = shooter_controller.getMechanismSetpointVelocity().get().in(RPM) / 60;
                    
                    // Check if shooter is within 2% of target speed
                    double tolerance = Math.abs(targetVelocityRPM * 0.05);
                    return Math.abs(currentVelocityRPM - targetVelocityRPM) <= tolerance;
                } catch (Exception e) {
                    System.out.println("Sinful, I know");
                    return false;
                }
            } else {
                return false;
            }
        }
    }

    /**
     * Creates a command to start the shooter at full speed.
     * 
     * @return command that runs shooter at full forward speed
     */
    public CCommand Shoot() {
        return cCommand("StartShooting")
                .onInitialize(() -> {
                    smc.startClosedLoopController();
                    state = ShooterState.On;
                    
                    shooter_controller.setMechanismVelocitySetpoint(ShooterSubsystemConstants.forwardsOnSpeeds);
                    isUsingStaticSpeed = true;
                    // In simulation, spawn a projectile when shooting starts so visuals match the command
                    if (RobotBase.isSimulation()) {
                        lastSpawnNs = System.nanoTime();
                    }
                })
                .onEnd(() -> {
                    state = ShooterState.Off;
                    smc.stopClosedLoopController();
                    shooter_motor_controller.setVoltage(0);
                    lastSpawnNs = 0;
                    isUsingStaticSpeed = true;
                })
                .isFinished(() -> {
                    if (Robot.isSimulation()) {
                        return intakeSubsystem.getHopperCount() == 0; // if hopper is empty
                    }
                    return false; // Some method to stop shooting irl, most likely current
                });
    }

    /**
     * Creates a command to start the shooter at full speed.
     * 
     * @return command that runs shooter at full forward speed
     */
    public CCommand manualSetpointShoot(int rpm) {
        return cCommand("StartShooting")
                .onInitialize(() -> {
                    targetRPM = RPM.of(rpm);
                    smc.startClosedLoopController();
                    state = ShooterState.On;
                    isUsingStaticSpeed = false;
                    // In simulation, spawn a projectile when shooting starts so visuals match the command
                    if (RobotBase.isSimulation()) {
                        lastSpawnNs = System.nanoTime();
                    }
                })
                .onExecute(() -> {
                    shooter_controller.setMechanismVelocitySetpoint(targetRPM);
                })
                .onEnd(() -> {
                    state = ShooterState.Off;
                    smc.stopClosedLoopController();
                    shooter_motor_controller.setVoltage(0);
                    lastSpawnNs = 0;
                    isUsingStaticSpeed = true;
                })
                .isFinished(() -> {
                    if (Robot.isSimulation()) {
                        return intakeSubsystem.getHopperCount() == 0; // if hopper is empty
                    }
                    return false; // Some method to stop shooting irl, most likely current
                });
    }

    /**
     * Creates a command to start the shooter at full speed.
     * 
     * @return command that runs shooter at full forward speed
     */
    public CCommand ShootWithMappedSpeeds() {
        return cCommand("StartShooting")
                .onInitialize(() -> {
                    smc.startClosedLoopController();
                    state = ShooterState.On;
                    isUsingStaticSpeed = false;
                })
                .onExecute(() -> {
                    // Get distance to target from the drive subsystem
                    if (driveSubsystem == null) {
                        return;
                    }
                    double distanceToTarget = driveSubsystem.getPose().getTranslation().getDistance(FieldConstants.hubPosition.get());
                    // Look up target shooter speed from the interpolating map based on distance
                    double targetRPM = shooterSpeedMap.get(distanceToTarget);
                    shooter_controller.setMechanismVelocitySetpoint(RPM.of(targetRPM));
                })
                .onEnd(() -> {
                    isUsingStaticSpeed = true;
                    state = ShooterState.Off;
                    smc.stopClosedLoopController();
                    shooter_motor_controller.setVoltage(0);
                })
                .isFinished(() -> {
                    if (Robot.isSimulation()) {
                        return intakeSubsystem.getHopperCount() == 0; // if hopper is empty
                    } else {
                        return false;
                    }
                });
    }

    public boolean noBalls() {
        if (!ballSensor.isConnected()) {
            return false;
        }
        // Read proximity from the color sensor. REV's ColorSensorV3 returns an
        // integer proximity where larger values indicate closer objects.
        int proximity = 0;
        try {
            proximity = ballSensor.getProximity();
        } catch (Exception ex) {
            // If reading fails, assume we can't detect; return false (not sure)
            return false;
        }

        // ballPresent when proximity is at-or-above the configured threshold
        boolean ballPresent = proximity >= RobotConstants.ShooterSubsystemConstants.BALL_PROXIMITY_THRESHOLD;

        long now = System.nanoTime();
        if (ballPresent) {
            // update last seen timestamp and report that balls are present
            lastBallSeenNs = now;
            return false;
        }

        // If we've never seen a ball yet, start the timer now and consider balls present
        if (lastBallSeenNs == 0) {
            lastBallSeenNs = now;
            return false;
        }

        // If no ball has been seen for at least 1 second, report no balls
        final long ONE_SECOND_NS = 1_000_000_000L;
        return (now - lastBallSeenNs) >= ONE_SECOND_NS;
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
                    smc.stopClosedLoopController();
                    shooter_motor_controller.setVoltage(-4);
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
                    smc.stopClosedLoopController();
                    shooter_motor_controller.setVoltage(0);
                });
    }

    /**
     * Updates telemetry data for the shooter controller.
     * Called periodically by the command scheduler.
     */
    @Override
    public void periodic() {
        logSelf();

        // Update telemetry
        shooter_controller.updateTelemetry();
        // Update the cached proximity so other subsystems can read it from a static accessor
        try {
            lastProximity = ballSensor.isConnected() ? ballSensor.getProximity() : -1;
        } catch (Exception ex) {
            lastProximity = -1;
        }

        // Also log color sensor data (RGB and IR) to SmartDashboard for debugging/tuning
        try {
            if (ballSensor.isConnected()) {
                var color = ballSensor.getColor();
                SmartDashboard.putNumber("Sensors/HopperColorR", color.red);
                SmartDashboard.putNumber("Sensors/HopperColorG", color.green);
                SmartDashboard.putNumber("Sensors/HopperColorB", color.blue);
                // IR value (if available) and proximity
                try {
                    SmartDashboard.putNumber("Sensors/HopperIR", ballSensor.getIR());
                } catch (Throwable t) {
                    // Some firmware versions may not expose IR - ignore
                }
                SmartDashboard.putNumber("Sensors/HopperProximityCached", lastProximity);
                SmartDashboard.putBoolean("Sensors/HopperBallPresent", lastProximity >= RobotConstants.ShooterSubsystemConstants.BALL_PROXIMITY_THRESHOLD);
            } else {
                SmartDashboard.putNumber("Sensors/HopperColorR", -1);
                SmartDashboard.putNumber("Sensors/HopperColorG", -1);
                SmartDashboard.putNumber("Sensors/HopperColorB", -1);
                SmartDashboard.putNumber("Sensors/HopperIR", -1);
                SmartDashboard.putNumber("Sensors/HopperProximityCached", -1);
                SmartDashboard.putBoolean("Sensors/HopperBallPresent", false);
            }
        } catch (Exception ex) {
            // don't let dashboard logging affect periodic
        }
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
                try {
                    // Only spawn/launch projectiles in sim when either the Drive subsystem isn't present
                    // or when the robot is at the autodrive target with a small tolerance. Use 2% positional
                    // tolerance relative to mid-range and a small rotational tolerance (3.6 deg = 2% of 180deg).
                    boolean allowSpawn = true;
                    if (this.driveSubsystem != null) {
                        // Allow spawn if the robot is aimed at the hub within 3 degrees (override)
                        // Otherwise, if autodrive is active require being at the autodrive target distance.
                        if (this.driveSubsystem.isAimedAtHub(Math.toRadians(6.0))) {
                            // If we're pointing at the hub within 3°, always allow spawn.
                            allowSpawn = true;
                        } else if (this.driveSubsystem.isDriveToPoseActive()) {
                            // When autodrive is active, require the robot's distance to the hub be within a
                            // tolerance of the configured midRange used to generate the drive-to-curve target
                            // AND require the robot to be pointing at the hub within 3 degrees.
                            double midRange = RobotConstants.ShooterSubsystemConstants.midRange;
                            double posTol = midRange * 0.05; // 5% tolerance around midRange
                            double hubDist = this.driveSubsystem.getPose().getTranslation().getDistance(
                            RobotConstants.FieldConstants.hubPosition.get());
                            boolean distOk = Math.abs(hubDist - midRange) <= posTol;
                            boolean angleOk = this.driveSubsystem.isAimedAtHub(Math.toRadians(6.0));
                            allowSpawn = distOk && angleOk;
                        } else {
                            // Not autodriving and not aimed: allow normal spawning.
                            allowSpawn = true;
                        }
                    }

                    if (allowSpawn && removeBallFromHopper()) {
                        fuelSim.launchFuel(MetersPerSecond.of(8), Degrees.of(75.0), Degrees.of(180.0), Meters.of(0.9));
                        System.out.println("Shooting Balls");
                    } else {
                        System.out.println("Failed to shoot balls");
                    }
                } catch (IllegalStateException ex) {
                    // Robot not registered with fuelSim yet; ignore spawn attempt
                }
            }
        }
    }
}
