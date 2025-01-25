// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  SparkFlex m_bottom;
  SparkFlex m_top;

  private double m_topSpeed = 0;
  private double m_bottomSpeed = 0;

  private double m_simRPM = 0;

  public ShooterSubsystem() {
    SparkFlexConfig bottomConfig = new SparkFlexConfig();
    bottomConfig.idleMode(IdleMode.kCoast);
    bottomConfig.inverted(true);
    m_bottom =  new SparkFlex(ShooterConstants.kBottomShooterMotorPort, MotorType.kBrushless);
    m_bottom.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig topConfig = new SparkFlexConfig();
    topConfig.idleMode(IdleMode.kCoast);
    topConfig.inverted(true);
    m_top = new SparkFlex(ShooterConstants.kTopShooterMotorPort, MotorType.kBrushless);
    m_top.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void reset() {
    m_topSpeed = 0;
    m_bottomSpeed = 0;

    m_top.set(0);
    m_bottom.set(0);
  }

  public void setShootingSpeed(ShootSpeed speed) {
    switch (speed) {
      case Shooting:
        m_topSpeed = ShooterConstants.kShooterSpeedTop;
        m_bottomSpeed = ShooterConstants.kShooterSpeedBottom;
        break;
      case Amp:
        m_topSpeed = 0.2;
        m_bottomSpeed = 0.3;
      case Prep:
        m_topSpeed = ShooterConstants.kPrepShooterSpeed;
        m_bottomSpeed = ShooterConstants.kPrepShooterSpeed;
        break;
      case Off:
        m_topSpeed = 0.0;
        m_bottomSpeed = 0.0;
        break;
    }
  }

  public double returnCurrentSpeed() {
    if (m_topSpeed > 0.5 && m_simRPM < 15) m_simRPM++;
    else if (m_simRPM > 0) m_simRPM--;
    return Robot.isReal() ? m_bottom.getEncoder().getVelocity() : m_simRPM * 400;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("bottom Speed", m_bottomSpeed);
    SmartDashboard.putNumber("top Speed", m_bottom.getEncoder().getVelocity()); 

    m_bottom.set(m_bottomSpeed);
    m_top.set(m_topSpeed);
  }

  public static enum ShootSpeed {
    Shooting,
    Prep,
    Amp,
    Off
  }
}
