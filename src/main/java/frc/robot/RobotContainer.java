// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.RotateToAprilTag;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
public class RobotContainer {
    DigitalInput input = new DigitalInput(9);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final RobotCentric driveRR = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick buttons = new Joystick(1);
    private final SparkMax motor = new SparkMax(3, MotorType.kBrushless);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // private final  SendableChooser<Command> autoChooser;
    // SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(), new Rotation2d(logger.getCurrentRot()), drivetrain.getModulePositions(), drivetrain.getPoseLL());
    StructPublisher<Pose2d> publisher;
    Autos autos = new Autos(drivetrain, driveRR);




    List<Waypoint> waypoints =
      PathPlannerPath.waypointsFromPoses(
          new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
          new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
          new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)));

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    Pose2d targetPose = new Pose2d(8, 5, Rotation2d.fromDegrees(180));



    public RobotContainer() {
    // Add options to the chooser

    RobotModeTriggers.autonomous().whileTrue(autos.pathConnectingTest().cmd());
        // SmartDashboard.putNumber("Current Draw Climber", motor.getOutputCurrent());
        publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();
        configureBindings();
        }
    public void getInput() {
        
        
        publisher.set(drivetrain.getPose());
    }

    private void configureBindings() {
        System.out.println(input.get());
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () ->
                drive
                .withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(
                () -> 
                driveRR
                .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.start().onTrue(
          new InstantCommand(
            () -> drivetrain.resetGyro(0)
          )  
        );
        joystick.povUp().whileTrue(
            new RunCommand(
              () -> motor.set(0.15)
            )
          );
          joystick.povDown().whileTrue(
            new RunCommand(
              () -> motor.set(-0.15)
            )
          );
        joystick.povUp().whileFalse(
            new RunCommand(
                () -> motor.set(0.)
            )
        );
        joystick.povDown().whileFalse(
            new RunCommand(
                () -> motor.set(0.)
            )
        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
        new JoystickButton(buttons, 12).whileTrue(
            new SequentialCommandGroup(
                new RotateToAprilTag(drivetrain, driveRR, 3),
                new DriveToAprilTag(drivetrain, driveRR, 20, false, 0.),
                new RotateToAprilTag(drivetrain, driveRR, 2)
            )
        );
        joystick.a().and(joystick.povRight()).whileTrue(
            new DriveToAprilTag(drivetrain, driveRR, -20, true, -9)
        );
        joystick.a().and(joystick.povLeft()).whileTrue(
            new DriveToAprilTag(drivetrain, driveRR, 15, true, -7)
        );

        joystick.a().onTrue(AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
            ));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // public Command getAutonomousCommand() {

    //     return 
    // }
}
