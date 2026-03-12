// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import frc.robot.Libs.Elastic;
import frc.robot.Libs.MatchTime;

// 2026 field not yet available in Maple Sim - update when released
// import swervelib.simulation.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  private final MatchTime matchTime = new MatchTime(2026);
    private final StructPublisher<MatchTime> matchTimeTopic =
      NetworkTableInstance.getDefault()
        .getTable("Robot")
        .getStructTopic("MatchTime", MatchTime.struct)
        .publish();

  /**
   * Constructor for the Robot class.
   * Initializes the Maple Sim arena if running in simulation mode,
   * then creates the RobotContainer which sets up all subsystems and commands.
   */
  public Robot() {
    // Initialize Maple Sim arena BEFORE creating subsystems (if in simulation)
    if (isSimulation()) {
      SimulatedArena.getInstance();
      // Using generic arena - 2026 field will be added to Maple Sim in future update
      // SimulatedArena.overrideInstance(new Arena2026(...));  // Update when available
    }
    
    Dashboard.field2dInit();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    CameraServer.startAutomaticCapture();
    Elastic.selectTab("Autonomous");

    this.robotContainer = new RobotContainer();
    
    // Put CommandScheduler on SmartDashboard for monitoring
    SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
  }

  //runs continuously regardless of mode, execues the command scheduler
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    matchTime.update(MatchTime.kGameData2026.get());
    matchTimeTopic.set(matchTime);
    
    Dashboard.matchPhaseChange();
  }


  @Override
  public void disabledInit() {
    // Disable all subsystems in the Robot
  }

  /**
   * Runs continuously while the robot is in disabled mode.
   * Disabled mode occurs when the robot is powered on but not enabled by the Driver Station.
   * Called every 20ms during the disabled period.
   */
  @Override
  public void disabledPeriodic() {
      // Delegate disabled-periodic work to RobotContainer so Autos can update previews
      // only when the chooser selection changes.
  }

  /**
   * Called once when the robot exits disabled mode.
   * This occurs when transitioning from disabled to autonomous, teleop, or test mode.
   * Use this method to initialize or restart systems when transitioning to another mode.
   */
  @Override
  public void disabledExit() {}

  /**
   * Called once when autonomous mode starts.
   * Autonomous mode is the first 15 seconds of a match where the robot operates without driver input,
   * executing pre-programmed routines to score points or position itself strategically.
   * Retrieves the selected autonomous command from RobotContainer and schedules it for execution.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    System.out.println("Autonomous command: " + (autonomousCommand != null ? autonomousCommand.getName() : "None"));
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * Runs continuously during autonomous mode.
   * Autonomous mode is the first 15 seconds of a match where the robot operates without driver input.
   * Called every 20ms while the robot is in autonomous operation.
   */
  @Override
  public void autonomousPeriodic() {
    Dashboard.isHubActive();
  }

  /**
   * Called once when autonomous mode ends.
   * This occurs after the 15-second autonomous period concludes and before teleop begins.
   * Use this method to clean up autonomous-specific operations.
   */
  @Override
  public void autonomousExit() {
    Elastic.selectTab("Teleoperated");
  }
  
  /**
   * Called once when teleoperated (driver control) mode starts.
   * Teleop mode follows autonomous and lasts approximately 2 minutes and 15 seconds,
   * during which drivers control the robot using gamepads to score points and play defense.
   * Cancels any autonomous command that may still be running.
   */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * Runs continuously during teleoperated mode.
   * Teleop mode is the driver-controlled portion of the match lasting approximately 2 minutes and 15 seconds.
   * Called every 20ms while drivers have control of the robot.
   */
  @Override
  public void teleopPeriodic() {
    Dashboard.isHubActive();
  }

  /**
   * Called once when teleoperated mode ends.
   * This occurs when the match concludes or when the robot is disabled.
   * Use this method to clean up teleop-specific operations.
   */
  @Override
  public void teleopExit() {}

  /**
   * Called once when test mode starts.
   * Test mode is a special diagnostic mode for testing individual subsystems and components
   * in isolation using tools like the LiveWindow or custom test routines.
   * Cancels all running commands to ensure a clean testing environment.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Runs continuously during test mode.
   * Test mode is used for diagnostic testing and component verification outside of competition.
   * Called every 20ms while the robot is in test mode for diagnostic testing.
   */
  @Override
  public void testPeriodic() {}

  /**
   * Called once when test mode ends.
   * This occurs when exiting test mode to return to disabled or another operating mode.
   * Use this method to clean up test-specific operations.
   */
  @Override
  public void testExit() {}

  /**
   * Called once when simulation mode starts.
   * Simulation mode runs concurrently with other modes to provide physics-based simulation
   * of the robot and field, enabling testing and development without physical hardware.
   * The simulated arena is already initialized in the constructor, so no additional setup is needed.
   */
  @Override
  public void simulationInit() {
    // Arena already initialized in constructor
  }

  /**
   * Runs continuously during simulation mode.
   * Simulation mode provides a virtual environment for testing robot code without physical hardware.
   * Updates the Maple Sim arena to simulate robot physics and field interactions every 20ms.
   */
  @Override
  public void simulationPeriodic() {
    robotContainer.fuelSim.updateSim();
    // Step the projectile sim so projectiles are advanced and published for visualization
  }
}
