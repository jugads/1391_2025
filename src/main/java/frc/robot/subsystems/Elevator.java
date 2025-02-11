// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  // Hardware components for controlling the elevator's vertical movement
  SparkMax leftMotor = new SparkMax(kLeftMotorID, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(kRightMotorID, MotorType.kBrushless);
  double kStallSpeed = 0.;
  DutyCycleEncoder encoder = new DutyCycleEncoder(4);
  double setpoint = 0;
  // Limit switches to detect when elevator reaches its boundaries

  public Elevator() {
    encoder.setInverted(true);
    encoder.setInverted(true);
  }

  // Periodically updates SmartDashboard with elevator status information
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Right Current", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Left Current", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
    if (getElevatorDown()) {
      leftMotor.getEncoder().setPosition(0.);
      leftMotor.getEncoder().setPosition(0.);
    }
  }

  // Returns true when elevator is at bottom position
  public boolean getElevatorDown() {
    return leftMotor.getForwardLimitSwitch().isPressed();
  }

  // Returns true when elevator is at top position
  public boolean getElevatorUp() {
    return leftMotor.getReverseLimitSwitch().isPressed();
  }
  public double getSetpoint() {
    return MathUtil.clamp(setpoint, 0., 1.);
  }
  public void increaseSetpoint(double step) {
    setpoint += step;
  }
  public void setSetpoint(double target) {
    setpoint = target;
  }
  // Controls elevator movement using dual motors for balanced lifting
  public void runElevatorUp(double speed, Arm arm) {
    speed *= -1;
    if (speed > 0) {
    if (!(arm.getEncoderPosition() < 60)) {
    leftMotor.set(speed);
    rightMotor.set(-speed);
    }
    }
    else {
      leftMotor.set(speed);
    rightMotor.set(-speed);
    }
  }
  // Returns current elevator position using left motor's encoder
  public double getElevatorPosition() {
    return MathUtil.clamp(((-leftMotor.getEncoder().getPosition())/16.9), 0., 1.);
  }
  public double getStallCurrent() {
    return kStallSpeed;
  }
  public void stallElevator() {
    leftMotor.set(kStallSpeed);
    rightMotor.set(-kStallSpeed);
  }
}
