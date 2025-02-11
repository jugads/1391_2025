// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaeDefault;
import frc.robot.commands.AlignWithReef;
import frc.robot.commands.ArmDefault;
import frc.robot.commands.ArmToAngle;
import frc.robot.commands.AlgaeDefault;
import frc.robot.commands.ArmDefault;
import frc.robot.commands.ArmToAngle;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToReef;
import frc.robot.commands.ElevatorDefault;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ElevatorDefault;
import frc.robot.commands.KnuckleDefault;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RotateToAprilTag;
import frc.robot.commands.RunElevator;
import frc.robot.commands.Transfer;
import frc.robot.commands.Transfer;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.ReefPoses.kRED0_1;
import static frc.robot.Constants.ReefPoses.kRED10_11;
import static frc.robot.Constants.ReefPoses.kRED2_3;
import static frc.robot.Constants.ReefPoses.kRED4_5;
import static frc.robot.Constants.ReefPoses.kRED6_7;
import static frc.robot.Constants.ReefPoses.kRED8_9;
import static frc.robot.Constants.ReefPoses.kREDSOURCELEFT;
import static frc.robot.Constants.ReefPoses.kREDSOURCERIGHT;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Leds;
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    Pose2d currentSelfDrivingSetpoint = new Pose2d(0,0,new Rotation2d());
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick operator = new Joystick(1);
    private final CommandXboxController manual = new CommandXboxController(2);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // private final  SendableChooser<Command> autoChooser;
    // SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(), new Rotation2d(logger.getCurrentRot()), drivetrain.getModulePositions(), drivetrain.getPoseLL());
    StructPublisher<Pose2d> publisher;

    Arm arm = new Arm();
    AlgaeScorer algaeScorer = new AlgaeScorer();
    // Chute chute = new Chute();
    Elevator elevator = new Elevator();
    Knuckle knuckle = new Knuckle();
    Chute chute = new Chute();
    Leds leds = new Leds(new AddressableLED(5), new AddressableLEDBuffer(138), arm, knuckle, algaeScorer, chute);
    Autos autos = new Autos(drivetrain, driveRR, arm, elevator, knuckle, chute, leds);

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2*Math.PI, 4*Math.PI);


    public RobotContainer() {
    // Add options to the chooser
    // if (DriverStation.getAlliance().get() == Alliance.Blue) {drivetrain.getPigeon2().setYaw(0);}
   // else if (DriverStation.getAlliance().get() == Alliance.Red) {drivetrain.getPigeon2().setYaw(180);}
    RobotModeTriggers.autonomous().whileTrue(autos.fbranchanddbranch().cmd());
        // SmartDashboard.putNumber("Current Draw Climber", motor.getOutputCurrent());
        publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();
        configureBindings();
        }
    public void publishPose() {
        publisher.set(drivetrain.getPose());
        SmartDashboard.putNumber("GETTX", drivetrain.getTXFront());
    }

    private void configureBindings() {
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
        elevator.setDefaultCommand(
            new ElevatorDefault(elevator, arm)
        );
        knuckle.setDefaultCommand(
            new KnuckleDefault(knuckle)
        );
        algaeScorer.setDefaultCommand(new AlgaeDefault(algaeScorer));
        arm.setDefaultCommand(new ArmDefault(arm, arm.getEncoderPosition()));
        chute.setDefaultCommand(new InstantCommand(() -> chute.stopMotor(), chute));
        leds.setDefaultCommand(
            new InstantCommand(() ->
            leds.setDef(true), leds)
        );

        //DRIVER ------------------------------------------------------------------------------
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(
                () -> 
                driveRR
                .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );
        joystick.rightTrigger().whileTrue(
        drivetrain.applyRequest(
            () ->
            drive
            .withVelocityX(joystick.getLeftY() * MaxSpeed*0.3) // Drive forward with negative Y (forward)
            .withVelocityY(joystick.getLeftX() * MaxSpeed*0.3) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate*0.3) // Drive counterclockwise with negative X (left)
        ));
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        joystick.start().onTrue(
          new InstantCommand(
            () -> drivetrain.resetGyro(0)
          )  
        );
      
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(new InstantCommand(()->
            drivetrain.getPigeon2().setYaw(180)
        ));
        // joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.rightTrigger().whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(1.01)),
        //         new ArmToAngle(arm, 155),
        //         new RunCommand(() -> knuckle.setKnuckleMotorLow())
        //     )
        // );
        // operator.start().whileTrue(new RunCommand(() -> chute.runMotor(-0.4), chute).until(() -> chute.hasCoral()));
        joystick.leftTrigger().whileTrue(new ParallelCommandGroup(
            new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setSetpoint(0.73)),
            new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.68),
            new ArmToAngle(arm, 15).until(() -> arm.getEncoderPosition()<18),
            // new RunCommand(() -> System.out.println("Hello I work")),
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
            new RunCommand(() -> leds.setDef(false), leds)));
        joystick.leftBumper().whileTrue(new RunCommand(()->knuckle.score(), knuckle));
        joystick.y().whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.2)),
                new ArmToAngle(arm, 180),
                new RunCommand(() -> knuckle.setKnuckleMotorHigh
                (), knuckle)
            )
        );
        joystick.povLeft().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0.75) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        joystick.povRight().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(-0.75) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        joystick.povUp().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0.75) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        joystick.povDown().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(-0.75) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        joystick.b().whileTrue(AutoBuilder.pathfindToPose(kREDSOURCERIGHT, constraints));
        //joystick.x().whileTrue(AutoBuilder.pathfindToPose(kREDSOURCELEFT, constraints));
        
        // joystick.b().whileTrue(AutoBuilder.pathfindToPose(kRED0_1, constraints));
        //OPERATOR --------------------------------------------------------------------
        // operator.y().whileTrue(new ParallelCommandGroup(new RunCommand(() -> knuckle.setKnuckleMotorHigh(), knuckle), new RunCommand(() -> chute.runMotor(-0.3), chute)));
        // joystick.x().whileTrue(new RunCommand(() -> knuckle.score(), knuckle));
        // operator.rightBumper().whileTrue(new RunCommand(() -> elevator.increaseSetpoint(-0.005)));
        // operator.leftBumper().whileTrue(new RunCommand(() -> elevator.increaseSetpoint(0.005)));
        // operator.rightTrigger().whileTrue(new RunCommand(() -> arm.runMotor(0.1), arm));
        // operator.leftTrigger().whileTrue(new RunCommand(() -> arm.runMotor(-0.05), arm));
        // operator.a().whileTrue(new RunCommand(() -> algaeScorer.runAlgaeScorer(0.7), algaeScorer));
        // operator.b().whileTrue(new RunCommand(() -> algaeScorer.runAlgaeScorer(-1.), algaeScorer));
        // operator.a().onTrue(AutoBuilder.pathfindToPose(targetPose, constraints, 0.0));
        new JoystickButton(operator, kL1).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.63)),
                new ArmToAngle(arm, 80),
                new RunCommand(() -> knuckle.setKnuckleMotorLow())
            )
        );
        new JoystickButton(operator, kL2).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.25)),
                new ArmToAngle(arm, 160),
                new RunCommand(() -> knuckle.setKnuckleMotorLow())
            )
        );
        new JoystickButton(operator, kL3).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.53)),
                new ArmToAngle(arm, 160),
                new RunCommand(() -> knuckle.setKnuckleMotorLow())
            )
        );
        new JoystickButton(operator, kL4).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.98)),
                new ArmToAngle(arm, 160),
                new RunCommand(() -> knuckle.setKnuckleMotorLow())
            )
        );
        new JoystickButton(operator, kAutoAlignLeft).whileTrue(
            new DriveToReef(drivetrain, driveRR, true)
        );
        new JoystickButton(operator, kAutoAlignRight).whileTrue(
            new DriveToReef(drivetrain, driveRR, false)
        );
        new JoystickButton(operator, k0degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED6_7, constraints)
        );
        new JoystickButton(operator, k60degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED4_5, constraints)
        );
        new JoystickButton(operator, k120degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED2_3, constraints)
        );
        new JoystickButton(operator, k180degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED0_1, constraints)
        );
        new JoystickButton(operator, k240degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED10_11, constraints)
        );
        new JoystickButton(operator, k300degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED8_9, constraints)
        );
        // SmartDashboard.putNumber("null", operator.getY());
        //MANUAL -------------------------------------------------------------
      /*  manual.povUp().whileTrue(
        new RunCommand(() -> elevator.increaseSetpoint(0.005))
        );
        manual.povDown().whileTrue(
        new RunCommand(() -> elevator.increaseSetpoint(-0.005))
        );
        manual.leftTrigger().whileTrue(
        new RunCommand(() -> arm.runMotor(-0.05))
        );
        
        manual.rightTrigger().whileTrue(
        new RunCommand(() -> arm.runMotor(0.1))
        );
        drivetrain.registerTelemetry(logger::telemeterize); */
    } 
    public void elevatorReset() {
        elevator.setSetpoint(0);
    }

    // public Command getAutonomousCommand() {

    //     return 
    // }
}
