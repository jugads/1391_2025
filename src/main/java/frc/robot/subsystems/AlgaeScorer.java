// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import static frc.robot.Constants.AlgaeScorerConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeScorer extends SubsystemBase {
  // Motor controller for the algae scoring mechanism
  SparkMax motor;

  // Constructor initializes the brushless motor with specified ID
  public AlgaeScorer() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless);
  }

  // Continuously updates SmartDashboard with algae detection status
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Algae", hasAlgae());
    SmartDashboard.putNumber("Algae current", motor.getOutputCurrent());
    // This method will be called once per scheduler run
  }

  // Controls the algae scorer motor speed (-1.0 to 1.0)
  public void runAlgaeScorer(double speed) {
    motor.set(speed);
  }

  // Safely stops the motor by setting speed to zero
  public void stopMotor() {
    motor.set(0.);
  }

  // Retrieves the current draw from the motor for algae detection
  public double getAlgaeScorerCurrent() {
    return motor.getOutputCurrent();
  }

  // Determines if algae is present based on motor current threshold
  public boolean hasAlgae() {
    return getAlgaeScorerCurrent() > kCurrentThreshold;
  }
}
