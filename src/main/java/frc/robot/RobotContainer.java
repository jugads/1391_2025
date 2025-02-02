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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaeDefault;
import frc.robot.commands.ArmDefault;
import frc.robot.commands.ArmToAngle;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.ElevatorDefault;
import frc.robot.commands.KnuckleDefault;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RotateToAprilTag;
import frc.robot.commands.RunElevator;
import frc.robot.commands.Transfer;

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
    private final CommandXboxController operator = new CommandXboxController(1);

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
    Autos autos = new Autos(drivetrain, driveRR);

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2*Math.PI, 4*Math.PI);
    Pose2d targetPose = new Pose2d(8,5,Rotation2d.fromDegrees(180));

    public RobotContainer() {
    // Add options to the chooser
    
    RobotModeTriggers.autonomous().whileTrue(autos.pathConnectingTest().cmd());
        // SmartDashboard.putNumber("Current Draw Climber", motor.getOutputCurrent());
        publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();
        configureBindings();
        }
    public void publishPose() {
        publisher.set(drivetrain.getPose());
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
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
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
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(1.01)),
                new ArmToAngle(arm, 155),
                new RunCommand(() -> knuckle.setKnuckleMotorLow())
            )
        );
        operator.start().whileTrue(new RunCommand(() -> chute.runMotor(-0.4), chute).until(() -> chute.hasCoral()));
        joystick.y().whileTrue(new ParallelCommandGroup(new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setSetpoint(0.74)),
            new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.7),
            new ArmToAngle(arm, 15),
            new WaitUntilCommand(() -> arm.getEncoderPosition() < 18),
            new ParallelCommandGroup(
                new RunCommand(() -> knuckle.setKnuckleMotorHigh()),
                new RunCommand(() -> chute.runMotor(-0.45), chute)
            ).until(() -> knuckle.hasCoral()),
            new ParallelCommandGroup(
                new RunCommand(() -> knuckle.setKnuckleMotorHigh()),
                new InstantCommand(() -> elevator.setSetpoint(0.8))
            ).until(() -> elevator.getElevatorPosition() > 0.76),
            new ParallelCommandGroup(
            new ArmToAngle(arm, 50),
            new RunCommand(() -> knuckle.setKnuckleMotorHigh()))
            ), 
            new RunCommand(() -> leds.setDef(false), leds)));
        joystick.povRight().whileTrue(new ArmToAngle(arm, 90));
        
        


        //OPERATOR --------------------------------------------------------------------
        operator.y().whileTrue(new ParallelCommandGroup(new RunCommand(() -> knuckle.setKnuckleMotorHigh(), knuckle), new RunCommand(() -> chute.runMotor(-0.3), chute)));
        joystick.x().whileTrue(new RunCommand(() -> knuckle.score(), knuckle));
        operator.rightBumper().whileTrue(new RunCommand(() -> elevator.increaseSetpoint(-0.005)));
        operator.leftBumper().whileTrue(new RunCommand(() -> elevator.increaseSetpoint(0.005)));
        operator.rightTrigger().whileTrue(new RunCommand(() -> arm.runMotor(0.1), arm));
        operator.leftTrigger().whileTrue(new RunCommand(() -> arm.runMotor(-0.05), arm));
        operator.a().whileTrue(new RunCommand(() -> algaeScorer.runAlgaeScorer(0.7), algaeScorer));
        operator.b().whileTrue(new RunCommand(() -> algaeScorer.runAlgaeScorer(-1.), algaeScorer));
        // operator.a().onTrue(AutoBuilder.pathfindToPose(targetPose, constraints, 0.0));
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public void elevatorReset() {
        elevator.setSetpoint(0.);
    }

    // public Command getAutonomousCommand() {

    //     return 
    // }
}
