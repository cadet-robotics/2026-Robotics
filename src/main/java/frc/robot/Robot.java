// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
// 2026 field not yet available in Maple Sim - update when released
// import swervelib.simulation.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    // Initialize Maple Sim arena BEFORE creating subsystems (if in simulation)
    if (isSimulation()) {
      SimulatedArena.getInstance();
      // Using generic arena - 2026 field will be added to Maple Sim in future update
      // SimulatedArena.overrideInstance(new Arena2026(...));  // Update when available
    }
    this.robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // Disable all subsystems in the Robot
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // if (autonomousCommand != null) {
    //   autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    // Arena already initialized in constructor
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
  }
}
