// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private boolean haveNote = false;

  private CANSparkFlex m_intakeMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  private CANSparkFlex m_armMotor = new CANSparkFlex(IntakeConstants.kArmMotorID, MotorType.kBrushless);

  private PIDController m_armPID = new PIDController(0.5, 0, 0);

  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(IntakeConstants.kArmEncoderChannel);

  private Rev2mDistanceSensor m_distanceSensor = new Rev2mDistanceSensor(Port.kOnboard); // onboard I2C port;

  private double m_intakeSpeed = 0;
  private double m_armSetpoint = IntakeConstants.kIntakeLoweredAngle;

  /** Creates a new IntakeSubsystem */
  public IntakeSubsystem() {
    // m_armMotor.setInverted(IntakeConstants.kArmMotorInverted);
    // m_intakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);

    m_armEncoder.setPositionOffset(IntakeConstants.kArmEncoderOffset);
    m_armEncoder.setDistancePerRotation(360);

    m_intakeMotor.setIdleMode(IdleMode.kCoast);
    m_armMotor.setIdleMode(IdleMode.kBrake);

    m_armPID.setTolerance(0.05);
  }

  public void setArmPosition(ArmPosition position) {
    switch (position) {
      case Amp:
        m_armSetpoint = IntakeConstants.kIntakeAmpScoringAngle;
        break;
      case Extended:
        m_armSetpoint = IntakeConstants.kIntakeRaisedAngle;
        break;
      case Retracted:
        m_armSetpoint = IntakeConstants.kIntakeLoweredAngle;
      default:
        break;
    }
  }

  public boolean armAtSetpoint() {
    return m_armPID.atSetpoint();
  }

  public void intake() {
    m_intakeSpeed = IntakeConstants.kIntakeSpeed;
  }

  public void outtake() {
    m_intakeSpeed = -IntakeConstants.kIntakeSpeed;
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

    // m_armMotor.set(m_armPID.calculate(m_armEncoder.getDistance()));
    // m_intakeMotor.set((m_intakeSpeed >= 0 && haveNote) ? 0 : m_intakeSpeed);

    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getDistance());
    SmartDashboard.putBoolean("Have Note?", haveNote);
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
