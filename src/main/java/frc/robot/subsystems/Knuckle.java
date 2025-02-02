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
  String state = "null";
  double coralCount;
  boolean coralState = false;
  // Constructor initializes the brushless motor with specified ID
  public Knuckle() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless);
  }

  // Continuously updates SmartDashboard with coral detection status
  @Override
  public void periodic() {
    if (motor.getOutputCurrent() >= 15) {
      coralCount ++;
    }
    if (coralCount > 3 && motor.getOutputCurrent() <= 2) {
      coralCount = 0;
    }
    // if (hasCoral()) {
    //   setKnuckleMotorLow();
    // }
    // else {
    //   setKnuckleMotorHigh();
    // }
    SmartDashboard.putNumber("Coral Gripper Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Coral or Not", hasCoral());
    SmartDashboard.putNumber("Coral Count", coralCount);
    // This method will be called once per scheduler run
  }

  // Sets the knuckle motor to run at a predefined high speed
  public void setKnuckleMotorHigh() {
    motor.set(-kHighSpeed);
  }
  public boolean hasCoral() {
    if (coralCount > 5) {
      coralState = true;
    }
    else {
      coralState = false;
    }
    return coralState;
  }
  // Sets the knuckle motor to run at a predefined low speed
  public void setKnuckleMotorLow() {
    motor.set(-kLowSpeed);
    }
    public void score() {
      motor.set(-1.);
    }
  // Retrieves the current draw from the motor for coral detection
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  public String getState() {
    return state;
  }
  public void alterState(String newState) {
    state = newState;
  }
  public void stopMotor() {
    motor.set(0);
  }
  public void runMotor(double speed) {
    motor.set(speed);
  }
}
