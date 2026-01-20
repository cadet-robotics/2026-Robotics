// From https://github.com/br3ttb/Arduino-PID-AutoTune-Library/blob/master/PID_AutoTune_v0/PID_AutoTune_v0.cpp
package frc.robot.Libs;

import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleConsumer;

import static java.lang.Math.abs;

public class PIDAutoTuning {

    // Functional interfaces to read input and write output (replaces pointers from Arduino)
    private final DoubleSupplier inputSupplier;
    private final DoubleConsumer outputConsumer;
    private final DoubleSupplier outputSupplier; // To read current output value
    
    private double noiseBand = 0.5;
    private boolean running = false;
    private double oStep = 30;
    private boolean isMax;
    private boolean isMin;
    private double setpoint;
    private int controlType = 0; // Default to PI (0=PI, 1=PID)
    private double peak1, peak2;
    private double lastTime;
    private double sampleTime; // Changed to double for seconds (was int for millis in Arduino)
    private int nLookBack;
    private int peakType;
    private double[] lastInputs = new double[101];
    private double[] peaks = new double[10];
    private int peakCount;
    private boolean justchanged;
    private boolean justevaled;
    private double absMax, absMin;
    private double outputStart;
    private double Ku, Pu;

    /**
     * Constructor for PID AutoTuning
     * @param inputSupplier Supplies the current process variable value
     * @param outputConsumer Accepts the output value to control the process
     * @param outputSupplier Supplies the current output value (for initialization)
     */
    public PIDAutoTuning(DoubleSupplier inputSupplier, DoubleConsumer outputConsumer, DoubleSupplier outputSupplier) {
        this.inputSupplier = inputSupplier;
        this.outputConsumer = outputConsumer;
        this.outputSupplier = outputSupplier;
        this.setLookbackSec(10);
        this.lastTime = Timer.getFPGATimestamp();
    }

    public void cancel() {
        this.running = false;
    }

    public Optional<Integer> runtime() {
        this.justevaled = false;
        if(this.peakCount>9 && this.running)
        {
            this.running = false;
            this.finishUp();
            return Optional.of(1);
        }
        double now = Timer.getFPGATimestamp();

        // sampleTime is now in seconds (was millis in Arduino)
        if((now-this.lastTime)<this.sampleTime) return Optional.empty();
        this.lastTime = now;
        
        // Read current input value from supplier (replaces dereferencing pointer)
        double refVal = inputSupplier.getAsDouble();
        this.justevaled = true;
        
        if(!this.running) {
            this.peakType = 0;
            this.peakCount=0;
            this.justchanged=false;
            this.absMax=refVal;
            this.absMin=refVal;
            this.setpoint = refVal;
            this.running = true;
            
            // Read current output to use as baseline
            this.outputStart = outputSupplier.getAsDouble();
            
            // Write initial output to start oscillation
            outputConsumer.accept(this.outputStart + this.oStep);
        } else {
            if(refVal>this.absMax) {
                this.absMax = refVal;
            } else if(refVal<this.absMin) {
                this.absMin = refVal;
            }
        }

        // Oscillate the output based on the input's relation to the setpoint
        if(refVal>this.setpoint+noiseBand) {
            outputConsumer.accept(outputStart-oStep);
        } else if (refVal<setpoint-noiseBand) {
            outputConsumer.accept(outputStart+oStep);
        }

        this.isMax=true;
        this.isMin=true;

        for (int i=nLookBack-1; i>=0; i--) {
            double val = this.lastInputs[i];
            if(this.isMax) this.isMax = refVal>val;
            if(this.isMin) this.isMin = refVal<val;
            this.lastInputs[i+1] = this.lastInputs[i];
        }
        this.lastInputs[0] = refVal;
        if( this.nLookBack < 9 ) {  //we don't want to trust the maxes or mins until the inputs array has been filled
            return Optional.empty();
        }

        if(this.isMax) {
            if(this.peakType==0) {
                this.peakType=1;
            }
            if(this.peakType==-1) {  // Fixed: Should be "if", not "else if" (Arduino has two separate ifs)
                this.peakType = 1;
                this.justchanged = true;
                this.peak2 = this.peak1;
            }
            this.peak1 = now;
            this.peaks[this.peakCount] = refVal;

        } else if(this.isMin) {
            if(this.peakType == 0) {
                this.peakType = -1;
            }
            if(this.peakType == 1) {  // Fixed: Should be "if", not "else if" (Arduino has two separate ifs)
                this.peakType = -1;
                this.peakCount++;
                this.justchanged = true;
            }

            if(this.peakCount<10) {
                this.peaks[this.peakCount] = refVal;
            }
        }

        if(this.justchanged && this.peakCount > 2) { //we've transitioned.  check if we can autotune based on the last peaks
            double avgSeparation = (abs(this.peaks[this.peakCount-1]-this.peaks[this.peakCount-2])+abs(this.peaks[this.peakCount-2]-this.peaks[this.peakCount-3]))/2;
            if( avgSeparation < 0.05*(this.absMax-this.absMin))
            {
                this.finishUp();
                this.running = false;
                return Optional.of(1);

            }
        }
        this.justchanged = false;
        return Optional.empty();
    }

    public void finishUp() {
        // Restore original output value
        outputConsumer.accept(outputStart);
        
        // Calculate ultimate gain and period
        // we can generate tuning parameters!
        double amplitude = absMax - absMin;
        if (amplitude > 0) {  // Safety check to avoid division by zero
            Ku = 4 * (2 * oStep) / (amplitude * 3.14159);
        } else {
            Ku = 0;  // No oscillation detected, cannot compute gain
        }
        
        // Fixed: peak1 and peak2 are already in seconds (from Timer.getFPGATimestamp())
        // Arduino divides by 1000 because millis() returns milliseconds
        // We don't need to divide since our timestamps are already in seconds
        Pu = Math.abs(peak1-peak2);
        
        // Safety check for period
        if (Pu == 0) {
            Pu = 1;  // Avoid division by zero in Ki calculation
        }
    }

    public double getKp() {
        if (Ku == 0 || Pu == 0) return 0;  // Safety check
        return this.controlType == 1 ? 0.6 * this.Ku : 0.4 * this.Ku;
    }

    public double getKi() {
        if (Ku == 0 || Pu == 0) return 0;  // Safety check
        return this.controlType == 1? 1.2 * this.Ku / this.Pu : 0.48 * this.Ku / this.Pu;  // Ki = Kc/Ti
    }

    public double getKd() {
        if (Ku == 0 || Pu == 0) return 0;  // Safety check
        return this.controlType == 1? 0.075 * this.Ku * this.Pu : 0;  //Kd = Kc * Td
    }

    public void setOutputStep(double step) {
        this.oStep = step;
    }

    public double getOutputStep() {
        return this.oStep;
    }

    public void SetControlType(int type) {
        // 0=PI, 1=PID
        this.controlType = type;
    }
    public int getControlType() {
        return this.controlType;
    }

    public void setNoiseBand(double band)
    {
        this.noiseBand = band;
    }

    public double getNoiseBand() {
        return this.noiseBand;
    }

    public void setLookbackSec(int value) {
        if (value < 1) {
            value = 1;
        }

        if(value < 25) {
            this.nLookBack = value * 4;
            // Convert to seconds (Arduino uses 250 milliseconds = 0.25 seconds)
            this.sampleTime = 0.250;
        } else {
            this.nLookBack = 100;
            // Convert to seconds (Arduino uses value*10 milliseconds)
            this.sampleTime = value * 0.010;
        }
    }

    public int getLookbackSec() {
        // Convert back to seconds for return value
        return (int)(this.nLookBack * this.sampleTime);
    }

}
