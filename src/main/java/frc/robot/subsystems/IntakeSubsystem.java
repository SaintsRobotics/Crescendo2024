// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
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
  public boolean m_colorSensorToggle = Robot.isReal();
  private ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kMXP);

  private ArmPosition m_armPosition = ArmPosition.Retracted;

  private double m_intakeSpeed = 0;
  private double m_armSetpoint = IntakeConstants.kIntakeRaisedAngle;

  /** Creates a new IntakeSubsystem */
  public IntakeSubsystem() {
    m_armEncoder.setPositionOffset(IntakeConstants.kArmEncoderOffset);
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
    m_armSetpoint = getArmPosition();
  }

  public void setArmPosition(ArmPosition position) {
    m_armPosition = position;
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

  public boolean ampReady(){
    return m_armEncoder.getDistance() < IntakeConstants.kIntakeAmpScoringAngle + IntakeConstants.kIntakeAmpTolerance && m_armEncoder.getDistance() > IntakeConstants.kIntakeAmpScoringAngle - IntakeConstants.kIntakeAmpTolerance;
  }

  public boolean armAtSetpoint() {
    return m_armPID.atSetpoint();
  }

  public void intake() {
    m_intakeSpeed = IntakeConstants.kIntakeSpeed;
  }

  public void outtake() {
    m_intakeSpeed = -1;
  }

  public void outtakeAmp() {
    m_intakeSpeed = -0.55;
  }

  public void stopIntake() {
    m_intakeSpeed = 0;
  }

  /**
   * Gets distance from Color sensor
   */
  public int getColorProximity() {
    return m_colorSensor.getProximity();
  }

  @Override
  public void periodic() {
    haveNote = m_colorSensorToggle ? getColorProximity() > IntakeConstants.kProximityThreshold : false;

    if (m_armPosition == ArmPosition.Amp) {
      m_armPID.setTolerance(0.25);
      m_armPID.setP(0.005);
      m_armPID.setD(0.0003);
    } else {
      m_armPID.setTolerance(10);
      m_armPID.setP(0.002);
      m_armPID.setD(0);
    }

    double armMotorSpeed = MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance(), m_armSetpoint), -0.3, 0.3);
    m_armMotor.set(armMotorSpeed);
    m_intakeMotor.set(m_intakeSpeed);

    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getDistance());
    SmartDashboard.putNumber("Arm Absolute Angle", m_armEncoder.getAbsolutePosition());
    SmartDashboard.putBoolean("Have Note?", haveNote);
    // SmartDashboard.putNumber("pid output", armMotorSpeed);
    SmartDashboard.putNumber("Proximity", m_colorSensor.getProximity());
    SmartDashboard.putBoolean("Color Sensor Toggle", m_colorSensorToggle);
    SmartDashboard.putNumber("IR", m_colorSensor.getIR());
  }

  public boolean haveNote() {
    return haveNote;
  }

  public static enum ArmPosition {
    Extended,
    Retracted,
    Amp
  }

  /**
   * Toggles the usage of color sensor
   */

  public void colorSensorToggle() {
    m_colorSensorToggle = !m_colorSensorToggle;
  }
}
