// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KnuckleConstants;

import static frc.robot.Constants.KnuckleConstants.*;

public class Knuckle extends SubsystemBase {
  // Motor controller for the knuckle mechanism
  SparkMax motor;

  // Constructor initializes the brushless motor with specified ID
  public Knuckle() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless);
  }

  // Continuously updates SmartDashboard with coral detection status
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Detected", hasCoral());
    // This method will be called once per scheduler run
  }

  // Sets the knuckle motor to run at a predefined high speed
  public void setKnuckleMotorHigh() {
    motor.set(kHighSpeed);
  }

  // Sets the knuckle motor to run at a predefined low speed
  public void setKnuckleMotorLow() {
    motor.set(kLowSpeed);
  }

  // Retrieves the current draw from the motor for coral detection
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  // Determines if coral is present based on motor current threshold
  public boolean hasCoral() {
    return getCurrent() > kCurrentThreshold;
  }
  
  public void stopMotor() {
    motor.set(0);
  }
}
