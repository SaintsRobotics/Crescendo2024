// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  CANSparkFlex m_bottom = new CANSparkFlex(ShooterConstants.kBottomShooterMotorPort, MotorType.kBrushless);
  CANSparkFlex m_top = new CANSparkFlex(ShooterConstants.kTopShooterMotorPort, MotorType.kBrushless);

  private double m_topSpeed = 0;
  private double m_bottomSpeed = 0;

  public ShooterSubsystem() {
    m_bottom.setIdleMode(IdleMode.kCoast);
    m_top.setIdleMode(IdleMode.kCoast);

    m_bottom.setInverted(true);
    m_top.setInverted(true);
  }

  public void setShootingSpeed(ShootSpeed speed) {
    switch (speed){
      case Shooting:
        m_topSpeed = ShooterConstants.kShooterSpeed;
        m_bottomSpeed = ShooterConstants.kShooterSpeed;
        // System.out.println("shoot speed: " + ShooterConstants.kShooterSpeed);
        break;
      case Off:
        m_topSpeed = 0.0;
        m_bottomSpeed = 0.0;
        // System.out.println("shoot speed: " + 0);
        break;
    }
  }

  public double returnCurrentSpeed(){
    return m_bottom.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("bottom Speed", m_bottomSpeed);
    SmartDashboard.putNumber("top Speed", m_topSpeed);

    m_bottom.set(m_bottomSpeed);
    m_top.set(m_topSpeed);
  }

  public static enum ShootSpeed{
    Shooting,
    Off
  } 
}
