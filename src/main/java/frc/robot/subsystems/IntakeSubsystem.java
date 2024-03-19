// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private boolean haveNote = false;

  private CANSparkFlex m_intakeMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  private CANSparkFlex m_armMotor = new CANSparkFlex(IntakeConstants.kArmMotorID, MotorType.kBrushless);

  private PIDController m_armPID = new PIDController(0.002, 0, 0);

  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(IntakeConstants.kArmEncoderChannel);

  /** If true, the distance sensor will be used to determine if we have a note */
  private boolean m_distanceSensorToggle = Robot.isReal();
  private Rev2mDistanceSensor m_distanceSensor = m_distanceSensorToggle ? new Rev2mDistanceSensor(Port.kMXP) : null; // NavX I2C access port

  private double m_intakeSpeed = 0;
  private double m_armSetpoint = IntakeConstants.kIntakeRaisedAngle;

  /** Creates a new IntakeSubsystem */
  public IntakeSubsystem() {
    if (m_distanceSensorToggle) m_distanceSensor.setAutomaticMode(true);

    m_armEncoder.setPositionOffset(IntakeConstants.kArmEncoderOffset);
    SmartDashboard.putNumber("arm", m_armEncoder.getAbsolutePosition());
    m_armEncoder.setDistancePerRotation(360);

    m_intakeMotor.setIdleMode(IdleMode.kCoast);
    m_armMotor.setIdleMode(IdleMode.kBrake);

    m_armPID.setTolerance(10);

    m_armSetpoint = m_armEncoder.getDistance();
  }

  public void reset() {
    m_intakeMotor.set(0);
    m_armMotor.set(0);

    m_intakeSpeed = 0;
    m_armSetpoint = getDistanceSensor();
  }

  public void setArmPosition(ArmPosition position) {
    switch (position) {
      case Amp:
        m_armSetpoint = IntakeConstants.kIntakeAmpScoringAngle;
        break;
      case Extended:
        m_armSetpoint = IntakeConstants.kIntakeLoweredAngle;
        break;
      case Retracted:
        m_armSetpoint = IntakeConstants.kIntakeRaisedAngle;
      default:
        break;
    }

    m_armPID.setSetpoint(m_armSetpoint);
  }

  public double getArmPosition() {
    return m_armSetpoint;
  }

  public boolean armAtSetpoint() {
    return m_armPID.atSetpoint();
  }

  public void intake() {
    m_intakeSpeed = IntakeConstants.kIntakeSpeed;
  }

  public void outtake() {
    m_intakeSpeed = -IntakeConstants.kIntakeSpeed - 0.5;
  }

  public void stopIntake() {
    m_intakeSpeed = 0;
  }

  /**
   * Gets distance from Rev 2m sensor
   */
  private double getDistanceSensor() {
    if(m_distanceSensorToggle){
      if (m_distanceSensor.getRange() == -1) {
        m_distanceSensorToggle = false;
      }
      return m_distanceSensor.getRange();
    }
    return -1;
  }

  public boolean getDistanceSensorToggle() {
    return m_distanceSensorToggle;
  }

  /** Toggles whether the distance sensor is used */
  public void toggleDistanceSensor() {
    m_distanceSensorToggle = !m_distanceSensorToggle;
  }

  @Override
  public void periodic() {
    haveNote = getDistanceSensorToggle() ? getDistanceSensor() < IntakeConstants.kDistanceSensorThreshold : false;

    // Note: negative because encoder goes from 0 to -193 cuz weird
    double armMotorSpeed = MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance(), m_armSetpoint), -0.3, 0.3);
    m_armMotor.set(armMotorSpeed);
    m_intakeMotor.set(m_intakeSpeed);
    SmartDashboard.putNumber("intakespeed", m_intakeSpeed);

    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getDistance());
    SmartDashboard.putNumber("Arm Absolute Angle", m_armEncoder.getAbsolutePosition());
    SmartDashboard.putBoolean("Have Note?", haveNote);
    SmartDashboard.putNumber("distance sensor", m_distanceSensorToggle ? m_distanceSensor.getRange(Rev2mDistanceSensor.Unit.kInches) : -1);
    SmartDashboard.putNumber("pid output", armMotorSpeed);
    SmartDashboard.putNumber("Get offset", m_armEncoder.getPositionOffset());
  }

  public boolean haveNote() {
    return haveNote;
  }

  public static enum ArmPosition {
    Extended,
    Retracted,
    Amp
  }
}
