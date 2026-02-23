package frc.robot.Subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Libs.CCommand;
import frc.robot.Libs.CSubsystem;

public class Shaker extends CSubsystem {
    private SparkMax shaker_motor = new SparkMax(30, MotorType.kBrushless);
    
    public Shaker() {}

    public CCommand shake() {
        return cCommand("ShakerSubsystem.Shake")
            .onInitialize(() -> {
                shaker_motor.set(1);
            })
            .onEnd(() -> {
                shaker_motor.set(0);
            });
    }
}
