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
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private boolean haveNote = false;

  private CANSparkFlex m_intakeMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  private CANSparkFlex m_armMotor = new CANSparkFlex(IntakeConstants.kArmMotorID, MotorType.kBrushless);

  private PIDController m_armPID = new PIDController(0.002, 0, 0);

  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(IntakeConstants.kArmEncoderChannel);

  private Rev2mDistanceSensor m_distanceSensor = new Rev2mDistanceSensor(Port.kMXP); // onboard I2C port;

  private double m_intakeSpeed = 0;
  private double m_armSetpoint = IntakeConstants.kIntakeRaisedAngle;

  /** Creates a new IntakeSubsystem */
  public IntakeSubsystem() {
    m_distanceSensor.setAutomaticMode(true);

    m_armEncoder.setPositionOffset(IntakeConstants.kArmEncoderOffset);
    SmartDashboard.putNumber("arm", m_armEncoder.getAbsolutePosition());
    m_armEncoder.setDistancePerRotation(360);

    m_intakeMotor.setIdleMode(IdleMode.kCoast);
    m_armMotor.setIdleMode(IdleMode.kBrake);

    m_armPID.setTolerance(10);

    m_armSetpoint = m_armEncoder.getDistance();
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

  public double getArmPosition(){
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
  public double getDistanceSensor() {
    return m_distanceSensor.getRange();
  }

  @Override
  public void periodic() {
    haveNote = getDistanceSensor() < IntakeConstants.kDistanceSensorThreshold;

    //Note: negative because encoder goes from 0 to -193 cuz weird
    double setMotorSpeed = MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance(), m_armSetpoint), -0.4, 0.4);
    m_armMotor.set(setMotorSpeed);
    m_intakeMotor.set(m_intakeSpeed);
    SmartDashboard.putNumber("intakespeed", m_intakeSpeed);

    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getDistance());
    SmartDashboard.putBoolean("Have Note?", haveNote);
    SmartDashboard.putNumber("distance sensor", m_distanceSensor.getRange(Rev2mDistanceSensor.Unit.kInches));
    SmartDashboard.putNumber("pid output", setMotorSpeed);
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
