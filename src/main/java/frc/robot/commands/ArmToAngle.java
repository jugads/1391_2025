// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import static frc.robot.Constants.ArmConstants.*;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmToAngle extends Command {
    /** Creates a new ArmToAngle. */
    PIDController controller;
    Arm m_arm;
    double desiredArmAngle;
    ArmFeedforward feedforward;
    public ArmToAngle(Arm arm, double armAngle) {
      m_arm = arm;
        // desiredArmAngle = m_arm.getEncoderPosition();      
      desiredArmAngle = (armAngle/360)-0.25;

      controller = new PIDController(kPDynamic, kIDynamic, kDDynamic);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(desiredArmAngle);
    controller.setTolerance(1.);
    m_arm.setSetpoint(desiredArmAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Feedforward Calculation",feedforward.calculate(desiredArmAngle, 0.2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
