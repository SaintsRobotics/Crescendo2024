// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase {
  boolean deployed = true;

  private double m_pivotSpeed = 0;
  private double m_intakeSpeed = 0;

  CANSparkFlex m_topFeeder = new CANSparkFlex(0, MotorType.kBrushless);
  CANSparkFlex m_bottomFeeder = new CANSparkFlex(0, MotorType.kBrushless);
  CANSparkFlex m_pivotMotor = new CANSparkFlex(0, MotorType.kBrushless);

  PIDController m_pivotPID = new PIDController(0.5, 0, 0);

  DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(0);

  Rev2mDistanceSensor m_distanceSensor;

  /** Creates a new intake. */
  public IntakeSubsystem() {
    //m_pivotMotor.setInverted(Constants.IntakeConstants.kPivotMotorInverted);

    // TODO: honestly no idea waht to set
    //m_pivotEncoder.configMagnetOffset(Constants.IntakeConstants.kPivotEncoderOffset);
    //m_pivotEncoder.configAbsoluteSensorRange();

    m_topFeeder.setIdleMode(IdleMode.kBrake);
    m_bottomFeeder.setIdleMode(IdleMode.kBrake);

    m_pivotMotor.setIdleMode(IdleMode.kBrake);

    m_pivotPID.setTolerance(2);

    m_distanceSensor = new Rev2mDistanceSensor(Port.kOnboard); //onboard I2C port
    m_distanceSensor.setAutomaticMode(true);
  }

  public void turnOn(){
    m_pivotPID.setSetpoint(Constants.IntakeConstants.kIntakeLoweredAngle);

    deployed = true;
  }

  public void turnOff(){
    m_pivotPID.setSetpoint(Constants.IntakeConstants.kIntakeRaisedAngle);

    deployed = false;
  }

  /**
   * Gets distance from Rev 2m sensor
   * 
   */
  public double getDistanceSensor() {
    return m_distanceSensor.getRange();
  }

  /*
   * Helper function to calculate motor speeds
   * 
   */
  private void calculateSpeeds(){
    if (m_pivotEncoder.getAbsolutePosition() > IntakeConstants.kIntakeLoweredAngle && m_pivotEncoder.getAbsolutePosition() < IntakeConstants.kIntakeRaisedAngle){
      m_pivotSpeed = MathUtil.clamp(m_pivotPID.calculate(m_pivotEncoder.getAbsolutePosition()), -0.5, 0.5);
    }
    else{
      if (m_pivotPID.atSetpoint()){
        m_pivotSpeed = 0.0;
      }
    }
    
    if (m_pivotEncoder.getAbsolutePosition() < 15.0 && deployed){
      m_intakeSpeed = Constants.IntakeConstants.kIntakeSpeed; //TODO: i have no idea which speed
    } else {
      m_intakeSpeed = 0.0;
    }
  }

  @Override
  public void periodic() {
    calculateSpeeds();

    m_pivotMotor.set(m_pivotSpeed);

    m_bottomFeeder.set(m_intakeSpeed);
    m_topFeeder.set(m_intakeSpeed);
  }

  public double getPivotPosition(){
    return m_pivotEncoder.getAbsolutePosition();
  }

  public boolean readyForShooter(){
    return (m_pivotEncoder.getAbsolutePosition() < Constants.IntakeConstants.kIntakeRaisedAngle + 5) 
        || (m_pivotEncoder.getAbsolutePosition() > Constants.IntakeConstants.kIntakeRaisedAngle - 5);
  }
}
