// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  CANSparkFlex m_bottom = new CANSparkFlex(20, MotorType.kBrushless);
  CANSparkFlex m_top = new CANSparkFlex(35, MotorType.kBrushless);
  public ShooterSubsystem() {


  }
  public void spin(double speed){
    m_bottom.set(speed);
    m_top.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Speed", m_bottom.);
  }
}
