package frc.robot.Libs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.motorcontrollers.SmartMotorController;
import java.util.Optional;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * Wrapper for PID AutoTuning using the relay feedback method.
 * 
 * CRITICAL USAGE NOTES:
 * 1. You MUST call tuningLoop() periodically (e.g., in subsystem's periodic() method)
 * 2. Call changeAutoTune() to start/stop tuning
 * 3. The motor controller MUST allow voltage control during tuning - if configured for 
 *    CLOSED_LOOP mode, the PID will fight with the autotuner!
 * 4. SAFETY: Default aTuneStep=50V is DANGEROUS! Change to 3-6V for 12V systems
 * 5. Adjust aTuneNoise based on your mechanism's noise level
 */
public class APIDTunerSMCWrapper {

    private boolean tuning;
    private final SmartMotorController smc;
    private final PIDAutoTuning atuner;
    private double kp, ki, kd;
    private final String name;
    private final double aTuneNoise=1, aTuneStep = 50;
    private int aTuneLookBack = 20;
    
    // Store the current output for the autotuner to read/write
    private double autotunerOutput = 0;

    public APIDTunerSMCWrapper( String name, SmartMotorController smc ) {
        this.name = name;
        this.tuning = false;
        this.smc = smc;
        
        // Get initial PID values from the controller
        ProfiledPIDController closedLoopController = smc.getConfig().getClosedLoopController().get(); // Unsafe but should panic if not set
        this.kp = closedLoopController.getP();
        this.ki = closedLoopController.getI();
        this.kd = closedLoopController.getD();
        
        // Create autotuner with suppliers/consumers for I/O
        // Input: Read current velocity measurement
        // Output Consumer: Store output value (will be applied in tuningLoop)
        // Output Supplier: Provide current output value
        this.atuner = new PIDAutoTuning(
            () -> {
                // Get velocity in m/s (raw measurement)
                double velocity = this.smc.getMeasurementVelocity().in(MetersPerSecond);
                // Safety: return 0 if sensor reading is invalid
                return Double.isFinite(velocity) ? velocity : 0.0;
            },
            (value) -> this.autotunerOutput = value,                      // Output consumer
            () -> this.autotunerOutput                                     // Output supplier
        );
    }

    public void tuningLoop() {
        if (this.tuning) {
            // runtime() now returns an Optional<Integer> where empty means "still running".
            Optional<Integer> result = this.atuner.runtime();

            // NOTE: We do NOT apply output here - the calling code should read getCurrentOutput()
            // and decide how to control the motor (voltage, speed, etc.)
            // The autotuner uses relay feedback: output alternates between positive and negative

            if (result.isPresent()) {
                // A non-empty result indicates autotune has produced final PID values.
                this.tuning = false;
                
                // Update local copies of PID gains
                this.kp = this.atuner.getKp();
                this.ki = this.atuner.getKi();
                this.kd = this.atuner.getKd();
                
                // Output the final values
                this.outputPidValues();
            }
        }
    }

    public void changeAutoTune() {
        if (!tuning) {
            // Start autotuning
            this.atuner.setNoiseBand(aTuneNoise);
            this.atuner.setOutputStep(aTuneStep);
            this.atuner.setLookbackSec(this.aTuneLookBack);
            this.tuning = true;
        } else {
            // Cancel autotune
            this.atuner.cancel();
            this.tuning = false;
        }
    }

    /**
     * Check if autotuning is currently active
     * @return true if tuning is in progress
     */
    public boolean isTuning() {
        return this.tuning;
    }

    /**
     * Set the output step size (relay amplitude in volts)
     * @param step Output step in volts (recommend 3-6V for 12V systems)
     */
    public void setOutputStep(double step) {
        this.atuner.setOutputStep(step);
    }

    /**
     * Set the noise band (minimum oscillation amplitude to consider valid)
     * @param band Noise band in same units as input (m/s for velocity)
     */
    public void setNoiseBand(double band) {
        this.atuner.setNoiseBand(band);
    }

    /**
     * Set the lookback window size
     * @param seconds Lookback time in seconds
     */
    public void setLookbackSec(int seconds) {
        this.atuner.setLookbackSec(seconds);
    }

    /**
     * Get the current Kp value (from autotuner or last set value)
     */
    public double getKp() {
        return this.kp;
    }

    /**
     * Get the current Ki value (from autotuner or last set value)
     */
    public double getKi() {
        return this.ki;
    }

    /**
     * Get the current Kd value (from autotuner or last set value)
     */
    public double getKd() {
        return this.kd;
    }

    /**
     * Get the current output value from the autotuner
     * @return Current output signal (positive or negative)
     */
    public double getCurrentOutput() {
        return this.autotunerOutput;
    }

    void outputPidValues() {
        SmartDashboard.putNumber(this.name + "/kp", kp);
        SmartDashboard.putNumber(this.name + "/ki", ki);
        SmartDashboard.putNumber(this.name + "/kd", kd);
    }
}
