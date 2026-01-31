// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Main class for the robot application.
 * This class should not be instantiated and serves as the entry point for the robot program.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   * 
   * @param args command line arguments (not used)
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
