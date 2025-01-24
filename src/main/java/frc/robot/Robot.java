// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
//D license file in the root directory of this project.

package frc.robot;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private AddressableLEDBuffer buffer;
  private AddressableLED ledsObject = new AddressableLED(5);
  private Timer timer;
  private boolean increasing = true; // Tracks if brightness is increasing
private double brightness = 0;     // Current brightness (0-1 range)
private final double fadeSpeed = 0.1; // Adjust this value for fade speed
  private AutoChooser autoChooser;
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    // Initialize LEDs
    // Put the auto chooser on the dashboard
    // Shuffleboard.getTab("Autonomous").add(autoChooser);
    SmartDashboard.putData(autoChooser);
    // Schedule the selected auto during the autonomous period
     // PWM port 9
    // Initialize timer for animation
    // Create the auto chooser

  /**
   * Creates a new auto factory for this drivetrain with the given
   * trajectory logger.
   *
   * @param trajLogger Logger for the trajectory
   * @return AutoFactory for this drivetrain
   */
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    m_robotContainer.getInput();
  }

  @Override
  public void disabledInit() {
    timer = new Timer();
    timer.start();
    
    // DataLog log = DataLogManager.getLog();
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

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
  public void simulationPeriodic() {}
}
