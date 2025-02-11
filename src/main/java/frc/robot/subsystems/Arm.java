// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Arm extends SubsystemBase {
  // Hardware components for controlling the robot's arm mechanism
  SparkMax motor;
  DutyCycleEncoder encoder;
  double lastPosition;
  // Constructor initializes motor and encoder with specified ports from Constants
  public Arm() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless); 
    encoder = new DutyCycleEncoder(kEncoderPort);
    lastPosition = getEncoderPosition();
  }

  // Periodic method runs repeatedly, updates dashboard with arm status
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", getEncoderPosition());
    SmartDashboard.putNumber("Arm Speed Motor", motor.get());
    SmartDashboard.putNumber("Setpoint arm", lastPosition);
    // setSetpoint(getEncoderPosition());
    // This method will be called once per scheduler run
  }

  // Sets the motor speed (-1 to 1) to move the arm
  public void runMotor(double speed) {
    motor.set(speed);
  }

  // Stops arm movement by setting motor speed to 0
  public void stop() {
    motor.set(0.);
  }

  // Converts encoder reading to degrees (0-360)
  public double getEncoderPosition() {
    return (encoder.get())*360;
  }

  // Checks if arm is at the transfer position (0 degrees)
  public boolean atTransferAngle() {
    return Math.abs(getEncoderPosition() - kTransferAngle) <= 3;
  }

  public void setSetpoint(double encoderPosition) {
    lastPosition = encoderPosition;
  }
  public double getSetpoint() {return lastPosition;}
}
