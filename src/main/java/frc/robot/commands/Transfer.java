// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Knuckle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Transfer extends Command {
  /** Creates a new Transfer. */
  Knuckle knuckle;
  Hopper hopper;
  Elevator elevator;
  Arm arm;
  public Transfer(Knuckle knuckle, Elevator elevator, Hopper hopper, Arm arm) {
    this.hopper = hopper;
    this.knuckle = knuckle;
    this.elevator = elevator;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(knuckle, elevator, hopper, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   /* new SequentialCommandGroup(
     new InstantCommand(() -> elevator.setSetpoint(0.60)),
      new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.55),
      new ArmToAngle(arm, 5).until(() -> arm.getEncoderPosition()<10),
      new ParallelCommandGroup(
        new RunCommand(() -> hopper.runBeltMotor(1)),
        new RunCommand(() -> hopper.runWheelMotor(0.2)),
        new RunCommand(() -> knuckle.runMotor(0.8))
      ).until(() -> knuckle.hasCoral())
   );
    */
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
