// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_LED = new AddressableLED(LEDConstants.kLEDPort);
  private final AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
  private int m_rainbowFirstPixelHue = 0;

  private Random rand = new Random();

  /** Creates a new {@link LEDSubsystem}. */
  public LEDSubsystem() {
    m_LED.setLength(LEDConstants.kLEDLength); // 29
    m_LED.setData(m_LEDBuffer);
    m_LED.start();

    // Yellow when bot turns on.
    setLED(50, 50, 0);

    SmartDashboard.putString("led", m_LEDBuffer.getLED(1).toString());
  }

  /**
   * Sets the rgb value for all LEDs.
   * 
   * @param r Red 0-255
   * @param g Green 0-255
   * @param b Blue 0-255
   */
  public void setLED(int r, int g, int b) {

    // If we get an invalid value just set it to a rainbow
    if (r == 256 && g == 256 && b == 256) {
      disco();
      return;
    }

    else if (r == 257 && g == 257 && b == 257){
      golden();
      return;
    }

    else if (r == 258 && g == 258 && b == 258){
      disco();
      return;
    }

    for (var i = 0; i < LEDConstants.kLEDLength; i++) {
      m_LEDBuffer.setRGB(i, r, g, b);
    }
    m_LED.setData(m_LEDBuffer);
    SmartDashboard.putString("led", m_LEDBuffer.getLED(1).toString());
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
      // Set the value
      m_LEDBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 2;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    m_LED.setData(m_LEDBuffer);
    SmartDashboard.putString("led", m_LEDBuffer.getLED(1).toString());
  }

  private void golden() {
    // For every pixel
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 120;
      // Set the value
      m_LEDBuffer.setHSV(i, hue, 240, 180);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 1;
    // Check bounds
    m_rainbowFirstPixelHue %= 120;
    

    m_LED.setData(m_LEDBuffer);
    SmartDashboard.putString("led", m_LEDBuffer.getLED(1).toString());
  }

  private void disco() {
    // For every pixel
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 120;
      // Set the value
      m_LEDBuffer.setHSV(i, hue, rand.nextInt(50, 180), rand.nextInt(50, 180));
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += rand.nextInt(1, 8);
    // Check bounds
    m_rainbowFirstPixelHue %= 120;
    

    m_LED.setData(m_LEDBuffer);
    SmartDashboard.putString("led", m_LEDBuffer.getLED(1).toString());
  }
}