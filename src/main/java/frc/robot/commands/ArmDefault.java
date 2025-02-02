// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import static frc.robot.Constants.ArmConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmDefault extends Command {
  /** Creates a new ArmToAngle. */
  PIDController controller;
  Arm m_arm;
  double desiredArmAngle;

  public ArmDefault(Arm arm, double angle) {
    controller = new PIDController(kP,kI,kD);
    m_arm = arm;
    desiredArmAngle = angle;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Arm Controller", -controller.calculate(m_arm.getEncoderPosition()));
  m_arm.runMotor((Math.sin(Math.toRadians(m_arm.getEncoderPosition()-12)))* (-controller.calculate(m_arm.getEncoderPosition(), m_arm.getEncoderPosition())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
