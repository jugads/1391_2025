// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  // Hardware components for controlling the elevator's vertical movement
  SparkMax leftMotor = new SparkMax(kMotorID, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(kOtherMotorID, MotorType.kBrushless);
  
  // Limit switches to detect when elevator reaches its boundaries
  DigitalInput downSwitch = new DigitalInput(kDownLimitPort);
  DigitalInput upSwitch = new DigitalInput(kUpLimitPort);

  public Elevator() {}

  // Periodically updates SmartDashboard with elevator status information
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elevator Down", getElevatorDown());
    SmartDashboard.putBoolean("Elevator Up", getElevatorUp());
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
  }

  // Returns true when elevator is at bottom position
  public boolean getElevatorDown() {
    return downSwitch.get();
  }

  // Returns true when elevator is at top position
  public boolean getElevatorUp() {
    return upSwitch.get();
  }

  // Controls elevator movement using dual motors for balanced lifting
  public void runElevatorUp(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  // Returns current elevator position using left motor's encoder
  public double getElevatorPosition() {
    return leftMotor.getEncoder().getPosition();
  }
}
