// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Leds;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autos extends Command {
  AutoFactory autoFactory;
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric driveRR;
  Arm arm;
  Elevator elevator;
  Knuckle knuckle;
  Chute chute;
  Leds leds;
  
  
  /** Creates a new Autos. */
  public Autos(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRR, Arm arm, Elevator elevator, Knuckle knuckle, Chute chute, Leds leds) {
     // The drive subsystem
     this.drivetrain = drivetrain;
     this.arm = arm;
     this.elevator = elevator;
     this.knuckle = knuckle;
     this.chute = chute;
     this.leds = leds;

    autoFactory = this.drivetrain.createAutoFactory();


    // Use addRequirements() here to declare subsystem dependencies.
  }

 public Command testpath() {
  
  return Commands.sequence(
    new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetGyro(0)),
      new InstantCommand(() -> drivetrain.resetPose(new Pose2d(9.66354751586914, 4.0638532638549805, drivetrain.getPigeon2().getRotation2d())))
    ),
    autoFactory.trajectoryCmd("New Path"));
 }
 public AutoRoutine pathConnectingTest() {
  final AutoRoutine autoRoutine = autoFactory.newRoutine("Path Connecting Test");
  final AutoTrajectory path = autoRoutine.trajectory("BStart-FBranch");
  final AutoTrajectory path1 = autoRoutine.trajectory("BFBranch-Source-DBranch");
  final AutoTrajectory path2 = autoRoutine.trajectory("BDBranch-Source-CBranch");
  
  autoRoutine.active().onTrue(
    path.resetOdometry()
    .andThen(
      new SequentialCommandGroup(
        path.cmd(),
        new AlignWithReef(drivetrain, driveRR), // Make correct ID and make if then statement for bluevs red april tag
        new DriveToReef(drivetrain, driveRR, false), 
        new ParallelCommandGroup( //score L4
          new InstantCommand(() -> elevator.setSetpoint(0.98)),
          new ArmToAngle(arm, 160),
          new RunCommand(() -> knuckle.setKnuckleMotorLow())
          ).until(() -> elevator.getElevatorPosition() > 0.975 && arm.getEncoderPosition() > 157),
          new RunCommand(() -> knuckle.score()),
         new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.2)),
                new ArmToAngle(arm, 180)
            ),
        path1.cmd(),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setSetpoint(0.73)),
            new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.68),
            new ArmToAngle(arm, 15).until(() -> arm.getEncoderPosition()<18),
            new ParallelCommandGroup(
                new RunCommand(() -> knuckle.setKnuckleMotorHigh(), knuckle),
                new RunCommand(() -> chute.runMotor(-0.45), chute)
            ).until(() -> knuckle.hasCoral()),
            new ParallelCommandGroup(
                new RunCommand(() -> knuckle.setKnuckleMotorHigh()),
                new InstantCommand(() -> elevator.setSetpoint(0.8))
            ).until(() -> elevator.getElevatorPosition() > 0.76),
            new ParallelCommandGroup(
            new ArmToAngle(arm, 185),
            new RunCommand(() -> knuckle.setKnuckleMotorHigh())),
            new WaitUntilCommand(() -> arm.getEncoderPosition()>130),
            new InstantCommand(() -> elevator.setSetpoint(0.08))
            ),
            new RunCommand(() -> leds.setDef(false), leds)),
            new AlignWithReef(drivetrain, driveRR), // Make correct ID and make if then statement for bluevs red april tag
            new DriveToReef(drivetrain, driveRR, false), 
            new ParallelCommandGroup( //score L4
            new InstantCommand(() -> elevator.setSetpoint(0.98)),
            new ArmToAngle(arm, 160),
            new RunCommand(() -> knuckle.setKnuckleMotorLow())).until(() -> elevator.getElevatorPosition() > 0.975 && arm.getEncoderPosition() > 157),
            new RunCommand(() -> knuckle.score()),
            new ParallelCommandGroup(
              new InstantCommand(() -> elevator.setSetpoint(0.2)),
              new ArmToAngle(arm, 180)
          ),
            path2.cmd(),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.73)),
                new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.68),
                new ArmToAngle(arm, 15).until(() -> arm.getEncoderPosition()<18),
                new ParallelCommandGroup(
                    new RunCommand(() -> knuckle.setKnuckleMotorHigh(), knuckle),
                    new RunCommand(() -> chute.runMotor(-0.45), chute)
                ).until(() -> knuckle.hasCoral()),
                new ParallelCommandGroup(
                    new RunCommand(() -> knuckle.setKnuckleMotorHigh()),
                    new InstantCommand(() -> elevator.setSetpoint(0.8))
                ).until(() -> elevator.getElevatorPosition() > 0.76),
                new ParallelCommandGroup(
                new ArmToAngle(arm, 185),
                new RunCommand(() -> knuckle.setKnuckleMotorHigh())),
                new WaitUntilCommand(() -> arm.getEncoderPosition()>130),
                new InstantCommand(() -> elevator.setSetpoint(0.08))
                ),
                new RunCommand(() -> leds.setDef(false), leds)),
                new AlignWithReef(drivetrain, driveRR), // Make correct ID and make if then statement for bluevs red april tag
                new DriveToReef(drivetrain, driveRR, true), 
                new ParallelCommandGroup( //score L4
                new InstantCommand(() -> elevator.setSetpoint(0.98)),
                new ArmToAngle(arm, 160),
                new RunCommand(() -> knuckle.setKnuckleMotorLow())).until(() -> elevator.getElevatorPosition() > 0.975 && arm.getEncoderPosition() > 157),
                new RunCommand(() -> knuckle.score())

         
        // add commands for transfering and scoring
      )
    )
    
  );
  return autoRoutine;
 }
 public Command spin() {
  return Commands.sequence(
    new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetGyro(0)),
      new InstantCommand(() -> drivetrain.resetPose(new Pose2d(8.811853408813477,4.044926643371582, drivetrain.getPigeon2().getRotation2d())))
    ),
    autoFactory.trajectoryCmd("RStart-FBranch")
    // autoFactory.trajectoryCmd("FBranch-Source"),
    // autoFactory.trajectoryCmd("Source-DBranch"),
    // autoFactory.trajectoryCmd("DBranch-Source"),
    // autoFactory.trajectoryCmd("Source-CBranch")
    );
  
 }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
