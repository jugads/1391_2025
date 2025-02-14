// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.HopperConstants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  SparkMax beltMotor;
  SparkMax wheelMotor;
  DigitalInput beambrake;
  public Hopper() {
beltMotor = new SparkMax(kBeltMotorID, MotorType.kBrushless);
wheelMotor = new SparkMax(kWheelMotorID, MotorType.kBrushless);
beambrake = new DigitalInput(kBeamBreakPort);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
SmartDashboard.putBoolean("BeamBrake", hasCoralHopper());

  }
  public void runBeltMotor(double speed) {
    beltMotor.set(speed);
    }
  public void stopBeltMotor() {
    beltMotor.set(0);
  }
  public void runWheelMotor(double speed) {
    wheelMotor.set(speed);
    }
  public void stopWheelMotor() {
    wheelMotor.set(0);
  }
  public void bothAtSameTime(double wheelSpeed, double beltSpeed) {
    beltMotor.set(beltSpeed);
    wheelMotor.set(wheelSpeed);
  }
  public boolean hasCoralHopper() {
    return !beambrake.get();
  }
}
