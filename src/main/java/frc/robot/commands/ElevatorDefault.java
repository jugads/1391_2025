// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.ElevatorConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDefault extends Command {
  // The elevator subsystem and target height
  private final Elevator elevator;  
  // PID controller for height control
  private final PIDController pidController;

  /** Creates a new ElevatorToHeight command. */
  public ElevatorDefault(Elevator elevator) {
    this.elevator = elevator;    
    // Initialize PID controller with constants from Constants file
    this.pidController = new PIDController(kP, kI, kD);    
    // Require the elevator subsystem
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    double speed = pidController.calculate(elevator.getElevatorPosition(), elevator.getElevatorPosition());
    SmartDashboard.putNumber("Controller output", speed);
    // elevator.runElevatorUp(speed);
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the elevator is at the target height
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
