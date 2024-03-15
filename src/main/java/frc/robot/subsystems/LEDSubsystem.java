// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.ShootSpeed;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_LED = new AddressableLED(LEDConstants.kLEDPort);
  private final AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

  /** Creates a new {@link LEDSubsystem}. */
  public LEDSubsystem() {
    m_LED.setLength(LEDConstants.kLEDLength); // 29
    m_LED.setData(m_LEDBuffer);
    m_LED.start();

    // Blue when bot turns on.
    setLED(0, 0, 255);

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
    for (var i = 0; i < LEDConstants.kLEDLength; i++) {
      m_LEDBuffer.setRGB(i, r, g, b);
    }
    m_LED.setData(m_LEDBuffer);
    SmartDashboard.putString("led", m_LEDBuffer.getLED(1).toString());
  }

  public void addIntakeStatus(ArmPosition s) {
    switch (s) {
      case Extended:
        setLED(0, 255, 255);
        break;
      case Retracted:
        setLED(0, 250, 0);
        break;
    }
  }

//TODO: potentially streamline
  public void addShooterStatus(ShootSpeed s, double tspeed, double bspeed) {
    switch (s) {
      case Shooting:
        if (tspeed > ShooterConstants.kTopShooterSpeed && bspeed > ShooterConstants.kBottomShooterSpeed) {
          setLED(0, 255, 255);
        } else if (tspeed > ShooterConstants.kPreShooterSpeed && bspeed > ShooterConstants.kPreShooterSpeed) {
          setLED(255, 0, 170);
        } else {
          setLED(255, 0, 0);
        }
        break;

      case Halfway:
        if (tspeed > ShooterConstants.kPreShooterSpeed && bspeed > ShooterConstants.kPreShooterSpeed) {
          setLED(255, 0, 170);
        } else {
          setLED(255, 0, 0);
        }
        break;
      default:
        setLED(255, 0, 0);
        break;

    }
  }
}