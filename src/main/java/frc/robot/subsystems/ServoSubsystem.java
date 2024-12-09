// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Currently this subsystem is just in case anything goes wrong with the normal, default Servo
 * implementation in WPILib because the servo we have might not work with that pulse length
 * 
 * current stats of MG995 servo:
 * 4.8 to 7.2 V power and signal
 * 20ms (50 Hz) PWM period
 * 5 microsecond dead band width
 * Rotate about 120 degrees (60 in each direction)
 * 
 * 
 * different source claims that it "provides a running angle of about 180 degrees over a servo
 * pulse range of 600 us to 2400 us"
 * 
 * studica claims pulse width range of 500 to 250 us with a neutral position of 1500 us and a 
 * dead band width of 4 us
 * 
 * another source: https://components101.com/motors/mg995-servo-motor
 * 
 * WATCH OUT FOR MS VS US
 * a cycle for the servo has to be 20ms (50 Hz) so a pulse width of 1500 us would be 1.5 ms
 */
public class ServoSubsystem extends SubsystemBase {

  private PWM m_iloveservos = new PWM(1);

  private int m_pulseWidth = 1500;

  /** Creates a new ServoSubsystem. */
  public ServoSubsystem() {
    m_iloveservos.setBoundsMicroseconds(20000, 2500, 1500, 500, 20000);
    // m_iloveservos.setB
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_iloveservos.setPulseTimeMicroseconds(m_pulseWidth);
  }

  /** Set pulse width in MICROSECONDS (us) */
  public void setPulseWidth(int pulseWidth) {
    m_pulseWidth = pulseWidth;
  }
}
