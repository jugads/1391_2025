// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.kMaxAngularRate;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithReef extends Command {
  /** Creates a new AlignWithReef. */
  
  PIDController thetaController = new PIDController(0.199, 0, 0.0015);
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  double setpoint;
  public AlignWithReef(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    var ID = drivetrain.getTIDFront();
    this.setpoint = (Math.PI+(ID-7)*(Math.PI/3));
    if (this.setpoint > Math.PI) {
      this.setpoint -= 2*Math.PI;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.setSetpoint(setpoint);
    thetaController.setTolerance(Math.PI/180);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrain.getTVFront()) {
      drivetrain.setControl(
        drive
        .withRotationalRate(kMaxAngularRate * thetaController.calculate(getYaw()))
        .withVelocityX(0)
        .withVelocityY(0)
      );
    }
      SmartDashboard.putNumber("Alignment setpoint", thetaController.getSetpoint());
      SmartDashboard.putBoolean("Aligned", isFinished());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(thetaController.getSetpoint()-getYaw()) < Math.PI/45;
  }

  public double getYaw() {
    return drivetrain.getPigeon2().getRotation2d().getRadians();
}
}
