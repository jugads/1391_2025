// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.ElevatorConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToHeight extends Command {
  // The elevator subsystem and target height
  private final Elevator elevator;
  private final double targetHeight;
  
  // PID controller for height control
  private final PIDController pidController;

  /** Creates a new ElevatorToHeight command. */
  public ElevatorToHeight(Elevator elevator, double targetHeight) {
    this.elevator = elevator;
    this.targetHeight = targetHeight;
    
    // Initialize PID controller with constants from Constants file
    this.pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(0.001);
    
    // Require the elevator subsystem
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    pidController.reset();
    pidController.setSetpoint(targetHeight);
  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    double speed = pidController.calculate(elevator.getElevatorPosition());
    elevator.runElevatorUp(speed);
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    elevator.runElevatorUp(0);
  }

  // Returns true when the elevator is at the target height
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
