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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private boolean deployed = false;
  private boolean haveNote = false;

  private CANSparkFlex m_intakeMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  private CANSparkFlex m_armMotor = new CANSparkFlex(IntakeConstants.kArmMotorID, MotorType.kBrushless);

  private PIDController m_armPID = new PIDController(0.5, 0, 0);

  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(IntakeConstants.kArmEncoderChannel);

  private Rev2mDistanceSensor m_distanceSensor = new Rev2mDistanceSensor(Port.kOnboard); // onboard I2C port;

  private double m_intakeSpeed = 0;

  /** Creates a new IntakeSubsystem */
  public IntakeSubsystem() {
    // m_armMotor.setInverted(IntakeConstants.kArmMotorInverted);
    // m_intakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);

    m_armEncoder.setPositionOffset(IntakeConstants.kArmEncoderOffset);
    m_armEncoder.setDistancePerRotation(2 * Math.PI);

    m_intakeMotor.setIdleMode(IdleMode.kCoast);
    m_armMotor.setIdleMode(IdleMode.kBrake);

    m_armPID.setTolerance(0.05);

    // TODO: See if this is needed
    // m_distanceSensor.setAutomaticMode(true);
  }

  public void armExtend() {
    m_armPID.setSetpoint(IntakeConstants.kIntakeLoweredAngle);

    stopIntake();

    deployed = true;
  }

  public void armRetract() {
    m_armPID.setSetpoint(IntakeConstants.kIntakeRaisedAngle);

    stopIntake();

    deployed = false;
  }

  public void intake() {
    if (deployed && !haveNote) {
      m_intakeSpeed = IntakeConstants.kIntakeSpeed;
    }
  }

  public void outake() {
    if (!deployed && haveNote) {
      m_intakeSpeed = -IntakeConstants.kIntakeSpeed;
    }
  }

  public void stopIntake() {
    m_intakeSpeed = 0;
  }

  
  public void intakeTimedRun(double time){
    final Timer timer = new Timer();
    timer.reset();
    timer.start();
    while(timer.get() != time){
      m_intakeMotor.set(IntakeConstants.kIntakeSpeed);
    }
    m_intakeMotor.set(0);

  }
  /**
   * Gets distance from Rev 2m sensor
   * 
   */
  public double getDistanceSensor() {
    return m_distanceSensor.getRange();
  }

  @Override
  public void periodic() {
    haveNote = getDistanceSensor() < IntakeConstants.kDistanceSensorThreshold;

    // If we have a note and the arm is deployed, automatically bring it back in
    if (haveNote && deployed) {
      stopIntake();
      armRetract();
    }

    m_armMotor.set(m_armPID.calculate(m_armEncoder.getAbsolutePosition()));
    m_intakeMotor.set(m_intakeSpeed);

    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getAbsolutePosition());
    SmartDashboard.putBoolean("Arm Deployed?", deployed);
    SmartDashboard.putBoolean("Have Note?", haveNote);
  }

  public boolean readyToShoot() {
    return haveNote && !deployed && m_armPID.atSetpoint();
  }
}
