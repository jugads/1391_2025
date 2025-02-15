// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  AddressableLED leds;
  AddressableLEDBuffer buffer;
  Timer timer = new Timer();
  Arm arm;
  AlgaeScorer algaeScorer;
  Knuckle knuckle;
  //Chute chute;
  boolean def = true;
  boolean transferring = false;
  public Leds(AddressableLED leds, AddressableLEDBuffer buffer, Arm arm, Knuckle knuckle, AlgaeScorer algaeScorer) {
    this.leds = leds;
    this.buffer = buffer;
    this.arm = arm;
    this.algaeScorer = algaeScorer;
    this.knuckle = knuckle;
    //this.chute = chute;
    timer.start();
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
    double time = timer.get();
    int length = buffer.getLength();

    for (int i = 0; i < length; i++) {
      // Spread hues across LEDs and animate over time
      int hue = (int) ((time * 100 + (i * 360.0 / length)) % 360); // Adjust time multiplier for speed
      int saturation = 255; // Full saturation
      int value = 50;      // Full brightness
      buffer.setLED(i, Color.fromHSV(hue, saturation, value));
    }

    // Push updated LED data to the strip
    leds.setData(buffer);
  }
  
  else {
    
   
    LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, Color.kBlack, Color.kChartreuse);
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));

    pattern.applyTo(buffer);
    leds.setData(buffer);
    /*  double time = timer.get();
    int length = buffer.getLength();

    for (int i = 0; i < length; i++) {
        // Keep hues around chartreuse (~90°) and slightly oscillate over time
        int hue = 90; // Chartreuse (yellow-green)
        int saturation = 255; // Full saturation
        int value = (int) ((Math.sin(time * 3 + i * 0.5) * 0.5 + 0.5) * 255); // Wave effect
        buffer.setLED(i, Color.fromHSV(hue, saturation, value));
      } */
    
     
    if (arm.atTransferAngle()) {
      setAll(Color.kFirebrick);
    }
    else if (knuckle.hasCoral()) {
      setAll(Color.kWhiteSmoke);
    }
    else if (algaeScorer.hasAlgae()) {
      setAll(Color.kMediumAquamarine);
    }
    //else if (chute.hasCoral()) {
     // flash(Color.kWhiteSmoke);
   // }
    else if (knuckle.hasCoral() && algaeScorer.hasAlgae()) {
      setAll(Color.kChartreuse);
    }
   /* else if (def == false ) {
      flash(Color.kChartreuse);
    } */
    // else {
    //   flash(Color.kDarkRed);
    // }
  }
    // This method will be called once per scheduler run
  }

  public void setAll(Color color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color); // Set all LEDs to red
    }
    leds.setData(buffer);
  }

  // Sets the color of the top half of the LED strip
  public void setTop(Color color) {
    int midPoint = buffer.getLength() / 2;
    for (int i = midPoint; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
    leds.setData(buffer);
  }
  public void setDef(boolean set) {
    def=set;
  }
  // Sets the color of the bottom half of the LED strip
  public void setBottom(Color color) {
    int midPoint = buffer.getLength() / 2;
    for (int i = 0; i < midPoint; i++) {
      buffer.setLED(i, color);
    }
    leds.setData(buffer);
  }

  // Sets the color of the top third of the LED strip
  public void setTopThird(Color color) {
    int length = buffer.getLength();
    int startIndex = (length * 2) / 3;
    for (int i = startIndex; i < length; i++) {
      buffer.setLED(i, color);
    }
    leds.setData(buffer);
  }

  // Sets the color of the middle third of the LED strip
  public void setMiddleThird(Color color) {
    int length = buffer.getLength();
    int startIndex = length / 3;
    int endIndex = (length * 2) / 3;
    for (int i = startIndex; i < endIndex; i++) {
      buffer.setLED(i, color);
    }
    leds.setData(buffer);
  }

  // Sets the color of the bottom third of the LED strip
  public void setBottomThird(Color color) {
    int length = buffer.getLength();
    int endIndex = length / 3;
    for (int i = 0; i < endIndex; i++) {
      buffer.setLED(i, color);
    }
    leds.setData(buffer);
  }

  // Makes the LEDs flash by alternating between the specified color and black
  public void flash(Color color) {
    if ((int)(timer.get() * 3) % 2 == 0) {
      setAll(color);
    } else {
      setAll(Color.kBlack);
    }
  }

  public void flashBetween(Color color, Color color2) {
    if ((int)(timer.get() * 3) % 2 == 0) {
      setAll(color);
    } else {
      setAll(color2);
    }
  }
   
  
 
 
}
