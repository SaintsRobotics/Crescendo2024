// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

  private SparkFlex m_intakeMotor;
  private SparkFlex m_armMotor;

  private PIDController m_armPID = new PIDController(0.002, 0, 0);

  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(IntakeConstants.kArmEncoderChannel, 360, IntakeConstants.kArmEncoderOffset);

  /** If true, the distance sensor will be used to determine if we have a note */
  public boolean m_colorSensorToggle = Robot.isReal();
  private ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kMXP);

  private ArmPosition m_armPosition = ArmPosition.Retracted;

  private double m_intakeSpeed = 0;
  private double m_armSetpoint = IntakeConstants.kIntakeRaisedAngle;

  /** Creates a new IntakeSubsystem */
  public IntakeSubsystem() {
  SparkFlexConfig intakeConfig = new SparkFlexConfig();
  intakeConfig.idleMode(IdleMode.kCoast);

  m_armEncoder.setInverted(true);
  //m_armPID.enableContinuousInput(0, 360);

  m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  m_intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig armConfig = new SparkFlexConfig();
    armConfig.inverted(true);
    armConfig.idleMode(IdleMode.kBrake);
    m_armMotor = new SparkFlex(IntakeConstants.kArmMotorID, MotorType.kBrushless);
    m_armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);    

    m_armPID.setTolerance(10);

    m_armSetpoint = 10;
  }

  public void reset() {
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
    return ((m_armEncoder.get() < IntakeConstants.kIntakeAmpScoringAngle + 1) || (m_armEncoder.get() > IntakeConstants.kIntakeAmpScoringAngle - 1)); 
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

    m_armPID.setSetpoint(m_armSetpoint);
    double armMotorSpeed = m_armPID.atSetpoint() ? 0 : MathUtil.clamp(m_armPID.calculate(m_armEncoder.get()), -0.3, 0.3);

    SmartDashboard.putNumber("Motor Speed", armMotorSpeed);
    SmartDashboard.putNumber("Arm Angle", m_armEncoder.get());
    SmartDashboard.putBoolean("Have Note?", haveNote);
    // SmartDashboard.putNumber("pid output", armMotorSpeed);
    SmartDashboard.putNumber("Proximity", m_colorSensor.getProximity());
    SmartDashboard.putBoolean("Color Sensor Toggle", m_colorSensorToggle);
    SmartDashboard.putNumber("IR", m_colorSensor.getIR());

    m_armMotor.set(armMotorSpeed);
    m_intakeMotor.set(m_intakeSpeed);
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
