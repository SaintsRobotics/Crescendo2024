// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  CANSparkFlex m_bottom = new CANSparkFlex(ShooterConstants.kBottomShooterMotorPort, MotorType.kBrushless);
  CANSparkFlex m_top = new CANSparkFlex(ShooterConstants.kTopShooterMotorPort, MotorType.kBrushless);

  public ShooterSubsystem() {

  }

  public void spin(double speed) {
    m_bottom.set(speed);
    m_top.set(speed);
  }

  public void shooterTimedRun(double time, double speed){
    final Timer timer = new Timer();
    timer.reset();
    timer.start();
    while(timer.get() != time){
      m_bottom.set(speed);
      m_top.set(speed);
    }
    m_bottom.set(0);
    m_top.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Speed", m_bottom.);
  }
}