// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import static frc.robot.Constants.LEDConstants.*;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.commands.RunIntake;

import java.util.Map;

import edu.wpi.first.units.Units;

public class LEDS extends SubsystemBase {

  public final AddressableLED m_led;
  int totalBufferLength = 179; 
  //Create the buffer 
  private AddressableLEDBuffer ledStrip = new AddressableLEDBuffer(totalBufferLength); //180
  // AddressableLEDBufferView [] ledStripSegs = new AddressableLEDBufferView[ledSegLens.length];

// ScoringMechSensor getMechSensor = 

    final AddressableLEDBufferView LEDSegmentRightFrontBotom = ledStrip.createView(0,21);
    final AddressableLEDBufferView LEDSegmentRightFrontTop = ledStrip.createView(22,43); 
    final AddressableLEDBufferView LEDSegmentRightRearTop = ledStrip.createView(44,65).reversed();
    final AddressableLEDBufferView LEDSegmentRightRearBotom = ledStrip.createView(66,88).reversed();
    final AddressableLEDBufferView LEDSegmentLeftRearBotom = ledStrip.createView(89,111);
    final AddressableLEDBufferView LEDSegmentLeftRearTop = ledStrip.createView(112,133);
    final AddressableLEDBufferView LEDSegmentLeftFrontTop = ledStrip.createView(134,155).reversed();
    final AddressableLEDBufferView LEDsegmentLeftFrontBotom = ledStrip.createView(156,178).reversed(); 
  
  

  // This contains the starting index for the next view to be created

  // Create an LED pattern that sets the strip to color 
  LEDPattern red = LEDPattern.solid(Color.kRed);
  LEDPattern orange_red = LEDPattern.solid(Color.kOrangeRed);
  LEDPattern orange = LEDPattern.solid(Color.kOrange);
  LEDPattern yellow = LEDPattern.solid(Color.kYellow);
  LEDPattern blue = LEDPattern.solid(Color.kBlue);
  LEDPattern purple = LEDPattern.solid(Color.kPurple);
  LEDPattern pink = LEDPattern.solid(Color.kPink);
  LEDPattern green = LEDPattern.solid(Color.kGreen);
  LEDPattern black = LEDPattern.solid(Color.kBlack);


  public LEDS() {
    // Add up the lengths
    // for (int len : ledSegLens){
  //     totalBufferLength += len;
  // }
  // Create a view corresponding to each of the elements in ledSegLens
  // The parameters to createView are the start-index and the end-index.
  // The end-index is one less than the start-index plus the length.
  // Note that the "inxNext+=ledSegLens[seg]" updates inxNext to start-inx of the next view

  //  int inxNext = 0;
  //   for (int seg =0; seg<ledSegLens.length; seg++){
  //     ledStripSegs[seg] = ledStrip.createView(inxNext, ((inxNext+=ledSegLens[seg])-1));
  // }
    // PWM port 9

    
  //   public static final AddressableLEDBufferView LEDSegmentRightFrontBotom = ledStripSegs[0];
  // public static final AddressableLEDBufferView LEDSegmentRightFrontTop = ledStripSegs[1]; 
  // public static final AddressableLEDBufferView LEDSegmentRightRearTop = ledStripSegs[2];
  // public static final AddressableLEDBufferView LEDSegmentRightRearBotom = ledStripSegs[3];
  // public static final AddressableLEDBufferView LEDSegmentLeftRearBotom = ledStripSegs[4];
  // public static final AddressableLEDBufferView LEDSegmentLeftRearTop = ledStripSegs[5];
  // public static final AddressableLEDBufferView LEDSegmentLeftFrontTop = ledStripSegs[6];
  // public static final AddressableLEDBufferView LEDsegmentFrontBotom = ledStripSegs[7]; 



  // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(LEDPWMport);
    // Reuse buffer
    m_led.setLength(179);

    // m_led.setColorOrder(ColorOrder.kBRG);//kitbot channel's issue 

    // Set the data
    m_led.setData(ledStrip);
    m_led.start();
  }

  @Override
  public void periodic() {
    // Set the LEDs
    m_led.setData(ledStrip);
  }

  /**
   * @param start The start point of the rainbow
   * @param end The end point of the rainbow
   */
  

  

public void SetSegmentByLevel(double length, Color ElevatorColor, Color AllianceColor, double brightness) {
  LEDPattern NewPattern;
  NewPattern = LEDPattern.steps(Map.of(0, ElevatorColor, length, AllianceColor));
    NewPattern.atBrightness(Units.Percent.of(brightness));
    NewPattern.applyTo(LEDSegmentLeftFrontTop); 
    NewPattern.applyTo(LEDSegmentLeftRearTop); 
    NewPattern.applyTo(LEDSegmentRightFrontTop); 
    NewPattern.applyTo(LEDSegmentRightRearTop);

    m_led.setData(ledStrip);
    m_led.start();

    SmartDashboard.putBoolean("Command run", true);
  }
public void SetSegmentByIntake(Color Algae, Color Coral, Color AllianceColor, double brightness, ScoringMechSensor m_sensor) {
  LEDPattern NewPattern; 
  if (!m_sensor.getScoringSensor()) {
  NewPattern = LEDPattern.solid(AllianceColor);
    }
    else { 
  NewPattern = LEDPattern.solid(Coral);
    }
    NewPattern.atBrightness(Units.Percent.of(brightness));
    NewPattern.applyTo(LEDSegmentLeftFrontTop); 
    NewPattern.applyTo(LEDSegmentLeftRearTop); 
    NewPattern.applyTo(LEDSegmentRightFrontTop); 
    NewPattern.applyTo(LEDSegmentRightRearTop);
    m_led.setData(ledStrip);
    m_led.start();
  }

public void SetSegmentByIntakeMech(Color Color, double brightness) {
  LEDPattern NewPattern;
  NewPattern = LEDPattern.solid(Color);
  
  NewPattern.atBrightness(Units.Percent.of(brightness));
  NewPattern.applyTo(LEDsegmentLeftFrontBotom);
  NewPattern.applyTo(LEDSegmentLeftRearBotom);
  NewPattern.applyTo(LEDSegmentRightFrontBotom);
  NewPattern.applyTo(LEDSegmentRightRearBotom);
  m_led.setData(ledStrip);
  m_led.start();

  }

public void ledOn(int r, int g, int b) {
  setRainbow(r, g, b);
}
public void ledOff() {
  setRainbow(0, 0, 0);
}

public void allOff(){
  for (var i = 0; i < ledStrip.getLength(); i++) {
     ledStrip.setRGB(i, 0, 0, 0);
  }
}

  public void setLed(int position, int r, int g, int b) {
    if (position < 0) position = 0;
    if (position > ledStrip.getLength()) position = ledStrip.getLength();
    if (position < ledStrip.getLength()) {
      ledStrip.setRGB(position, r, g, b);
    }
  }
  public void setSlashLed(int r, int g, int b) {
      for (int Index = 19; Index <= 33; Index ++){
        ledStrip.setRGB(Index, r, g, b);
      };
    }
  private void setRainbow(int r, int g, int b) {
    for (var i = 0; i < ledStrip.getLength(); i++) {
      if (i < ledStrip.getLength()) {
        ledStrip.setRGB(i, r, g, b);
      }
    }
  }
}