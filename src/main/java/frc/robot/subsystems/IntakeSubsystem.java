// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  CANSparkFlex m_intakeMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  CANSparkFlex m_armMotor = new CANSparkFlex(IntakeConstants.kArmMotorID, MotorType.kBrushless);

  PIDController m_intakeVeloPID = new PIDController(IntakeConstants.kIntakeP, IntakeConstants.kIntakeI,
      IntakeConstants.kIntakeD);
  PIDController m_armPID = new PIDController(IntakeConstants.kArmP, IntakeConstants.kArmI, IntakeConstants.kArmD);

  DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(IntakeConstants.kArmEncoderCh);

  /** Creates a new intake. */
  public IntakeSubsystem() {

  }

  // Starts intaking the disk
  public void intakeDisk() {
    m_intakeMotor.set(IntakeConstants.kIntakeSpeed);
  }

  //Stops rotating the intake
  public void stopIntaking() {
    m_intakeMotor.set(0);
  }

  /**
   * Rotates the arm to a given angle
   * @param angle motor to apply to intake
   * 
   */
  public void tiltToAngle(double angle) {
    m_armMotor.set(m_armPID.calculate(m_armEncoder.getAbsolutePosition(), angle));
  }

  //Stops rotating the arm
  public void stopRotating(){
    m_armMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake",m_intakeMotor.get()>0);

    // This method will be called once per scheduler run
  }
}
