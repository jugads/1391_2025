// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Knuckle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Transfer extends Command {
  /** Creates a new Transfer. */
  Knuckle knuckle;
  Chute chute;
  public Transfer(Knuckle knuckle, Chute chute) {
    this.chute = chute;
    this.knuckle = knuckle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(knuckle, chute);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    knuckle.alterState("searching");
    chute.runMotor(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
