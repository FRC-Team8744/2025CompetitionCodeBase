// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {
  private AddressableLED m_led;
  // private AddressableLED m_led2;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;


  public LEDS() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(179);
    m_led.setLength(m_ledBuffer.getLength());


    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }

  /**
   * @param start The start point of the rainbow
   * @param end The end point of the rainbow
   */
  public void rainbow(int start, int end) {
    // For every pixel
    for (var i = start; i < end; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

public void ledOn(int r, int g, int b) {
  setRainbow(r, g, b);
}
public void ledOff() {
  setRainbow(0, 0, 0);
}

public void allOff(){
  for (var i = 0; i < m_ledBuffer.getLength(); i++) {
     m_ledBuffer.setRGB(i, 0, 0, 0);
  }
}

  public void setLed(int position, int r, int g, int b) {
    if (position < 0) position = 0;
    if (position > m_ledBuffer.getLength()) position = m_ledBuffer.getLength();
    if (position < m_ledBuffer.getLength()) {
      m_ledBuffer.setRGB(position, r, g, b);
    }
  }
  public void setSlashLed(int r, int g, int b) {
      for (int Index = 19; Index <= 33; Index ++){
        m_ledBuffer.setRGB(Index, r, g, b);
      };
    }
  private void setRainbow(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i < m_ledBuffer.getLength()) {
        m_ledBuffer.setRGB(i, r, g, b);
      }
    }
  }
}