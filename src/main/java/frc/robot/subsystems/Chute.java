// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import static frc.robot.Constants.ChuteConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chute extends SubsystemBase {
  // Motor controller for the chute mechanism
  SparkMax motor;
  DigitalInput beamBreak  = new DigitalInput(kBeamBreakPort);
  double coralCount = 0;
  // Constructor initializes the chute's brushless motor with specified ID
  public Chute() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless);
  }

  // Periodic method runs repeatedly - currently empty but available for future monitoring
  @Override
  public void periodic() {
    if (!beamBreak.get()) {
      coralCount ++;
    }
    if (coralCount > 3 && beamBreak.get()) {
      coralCount = 0;
    }
    SmartDashboard.putBoolean("Chute has coral", hasCoral());
    SmartDashboard.putNumber("Coral Count", coralCount);
    if (!beamBreak.get()) {
      coralCount ++;
    }
    if (coralCount > 3 && beamBreak.get()) {
      coralCount = 0;
    }
    SmartDashboard.putBoolean("Chute has coral", hasCoral());
    SmartDashboard.putNumber("Coral Count", coralCount);
  }
  
  // Sets the chute motor to run at the specified speed (-1.0 to 1.0)
  public void runMotor(double speed) {
    motor.set(speed);
  }

  // Safely stops the chute motor by setting speed to zero
  public void stopMotor() {
    motor.set(0);
  }

  public boolean hasCoral() {
    return coralCount > 5;
  }
} */
