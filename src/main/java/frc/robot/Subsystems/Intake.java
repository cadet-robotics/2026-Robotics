package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ShooterState;
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
    private final SparkMax intakeMotorController = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);
    /** Configuration for the smart motor controller including PID, feedforward, and gearing. */
    private final SmartMotorControllerConfig smcConfig  = new SmartMotorControllerConfig(this)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withTelemetry("IntakeMotor",SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(9)))
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    /** Smart motor controller wrapper for the intake motor. */
    private final SmartMotorController intakeController = new SparkWrapper( intakeMotorController, DCMotor.getNeoVortex(1), smcConfig);

    /** Target state of the intake mechanism. */
    private IntakeState state = IntakeState.OFF;

    private final Supplier<ShooterState> getShooterState;
    private final Supplier<Boolean> isShooterUpToSpeed;
    
    private int hopperCount = 0;
    private int hopperMax = 23;

    public int getHopperCount() {
        return hopperCount;
    }
    
    public boolean isHopperFull() {
        return hopperCount >= hopperMax;
    }

    public void addToHopper() {
        if (hopperCount < hopperMax) {
            hopperCount++;
        }
        SmartDashboard.putNumber("Intake/HopperCount", hopperCount);
    }

    /**
     * Removes a ball from the hopper
     * @return whether the remove was successful (ie hopper was already empty)
     */
    public boolean removeFromHopper() {
        SmartDashboard.putNumber("Intake/HopperCount", hopperCount);
        if (hopperCount == 0) {
            return false;
        }
        hopperCount--;
        SmartDashboard.putNumber("Intake/HopperCount", hopperCount);
        return true;
    }

    // Optional Drive dependency for gating intake while shooting
    private final Drive driveSubsystem;

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
    public Intake(Shooter shooter_subsystem, Drive driveSubsystem) {
        setName("IntakeSubsystem");

        this.getShooterState = shooter_subsystem::getState;
        this.isShooterUpToSpeed = shooter_subsystem::isUpToSpeed;
        this.driveSubsystem = driveSubsystem;
    }

    public CCommand intakeToggler() {
        return cCommand().onInitialize(() -> {
            if (state == IntakeState.OFF) {
                state = IntakeState.ON;
            } else {
                state = IntakeState.OFF;
            }
        });
    }

    /**
     * Creates a command to turn the intake on.
     * 
     * @return command that sets intake state to On
     */
    public CCommand IntakeOn() {
        return cCommand().onInitialize(() -> {
            state = IntakeState.ON;
        }).onEnd(() -> {
            state = IntakeState.OFF;
        });
    }

    /**
     * Creates a command to turn the intake off.
     * 
     * @return command that sets intake state to Off
     */
    public CCommand IntakeOff() {
        return cCommand().onInitialize(() -> {
            state = IntakeState.OFF;
        });
    }

    /**
     * Creates a command to set the intake to barf.
     * 
     * @return command that sets intake state to Barf
     */
    public CCommand IntakeBarf() {
        return cCommand().onInitialize(() -> {
            state = IntakeState.BARF;
        }).onEnd(() -> {
            state = IntakeState.OFF;
        });
    }

    /**
     * Updates telemetry data for the intake motor controller.
     * Called periodically by the command scheduler.
     */
    @Override
    public void periodic() {
        logSelf();
        intakeController.updateTelemetry();

        // Run intake if manually commanded OR if shooter is on and up to speed
        boolean allowedByDrive = true;
        boolean allowedByAim = true;
        if (driveSubsystem != null) {
            // if drive-to-pose input stream is active, require being at the pose
            if (driveSubsystem.isDriveToPoseActive()) {
                allowedByDrive = true;
            }
            // if aim mode input stream is active, require being aimed at hub within tolerance
            if (driveSubsystem.isAimModeActive()) {
                allowedByAim = driveSubsystem.isAimedAtHub(Math.toRadians(6.0)); // ~6 deg tolerance
            }
        }

        if ( this.state == IntakeState.ON || ((this.getShooterState.get() == ShooterState.On && this.isShooterUpToSpeed.get()) && allowedByDrive && allowedByAim) ) {
            this.intakeController.setVoltage(Volts.of(11));
        } else if ( this.state == IntakeState.BARF ) {
            this.intakeController.setVoltage(Volts.of(-12));
        } else {
            this.intakeController.setVoltage(Volts.of(0));
        }
        // Publish whether the simulation intake condition is active
        // (matches the supplier used by FuelSim.registerIntake)
        boolean simIntakeActive = (this.state == IntakeState.ON) && (!this.isHopperFull());
        SmartDashboard.putBoolean("Intake/SimActive", simIntakeActive);
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