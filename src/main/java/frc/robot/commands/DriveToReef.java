// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.kMaxSpeed;
import static frc.robot.Constants.KnuckleConstants.kHighSpeed;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToReef extends Command {
  /** Creates a new DriveToReef. */
  PIDController xController = new PIDController(0.5, 0., 0);
  PIDController yController = new PIDController(0.55, 0., 0);
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  public DriveToReef(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(14.3);
    yController.setSetpoint(4.);
    xController.setTolerance(0.05);
    yController.setTolerance(0.005);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(drive
    .withVelocityX(-kMaxSpeed*xController.calculate(drivetrain.getPose().getX()))
    .withVelocityY(-kMaxSpeed * yController.calculate(drivetrain.getPose().getY()))
    .withRotationalRate(0.)
    );
    SmartDashboard.putNumber("X value", xController.calculate(drivetrain.getPose().getX()));
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
