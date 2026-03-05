package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ShooterState;
import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;

/**
 * Intake subsystem that controls the mechanism for collecting game pieces.
 * Manages motor control for intake operations including on, off, and reverse states.
 */
public class Intake extends CSubsystem {
    /** Motor controller for the intake mechanism. */
    private final SparkMax intakeMotorController = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);

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
            switch(state) {
                case OFF:
                    state = IntakeState.ON;
                    intakeMotorController.setVoltage(10);
                    break;
                case ON:
                case BARF:
                    state = IntakeState.OFF;
                    intakeMotorController.setVoltage(0);
                    break;
            }
        });
    }

    /**
     * Creates a command to turn the intake on.
     * 
     * @return command that sets intake state to On
     */
    public CCommand IntakeIn() {
        return cCommand().onInitialize(() -> {
            state = IntakeState.ON;
            intakeMotorController.setVoltage(10);
        }).onEnd(() -> {
            state = IntakeState.OFF;
            intakeMotorController.setVoltage(0);
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
            intakeMotorController.setVoltage(0);
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
            intakeMotorController.setVoltage(-10);
        }).onEnd(() -> {
            state = IntakeState.OFF;
            intakeMotorController.setVoltage(0);
        });
    }

    /**
     * Updates telemetry data for the intake motor controller.
     * Called periodically by the command scheduler.
     */
    @Override
    public void periodic() {
        logSelf();

        if (Robot.isSimulation()) {
            // Publish whether the simulation intake condition is active
            // (matches the supplier used by FuelSim.registerIntake)
            boolean simIntakeActive = (this.state == IntakeState.ON) && (!this.isHopperFull());
            SmartDashboard.putBoolean("Intake/SimActive", simIntakeActive);
            switch (state) {
                case ON:
                    SmartDashboard.putString("Intake/intakeState","on" );
                    break;
                case OFF:
                    SmartDashboard.putString("Intake/intakeState", "off");
                    break;
                case BARF:
            }
        }
    }
}