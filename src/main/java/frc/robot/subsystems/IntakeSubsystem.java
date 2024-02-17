// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

//creates new motors and pid controllers lmao
public class IntakeSubsystem extends SubsystemBase {
  CANSparkFlex intakeMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  CANSparkFlex armMotor = new CANSparkFlex(IntakeConstants.kArmMotorID, MotorType.kBrushless);
  
  PIDController intakeVeloPID = new PIDController(IntakeConstants.kIntakeP,IntakeConstants.kIntakeI,IntakeConstants.kIntakeD);
  PIDController armPID = new PIDController(IntakeConstants.kArmP,IntakeConstants.kArmI,IntakeConstants.kArmD);
  DutyCycleEncoder armEncoder = new DutyCycleEncoder(IntakeConstants.kArmEncoderCh);
  
  /** Creates a new intake. */
  public IntakeSubsystem() {
    
  }
    /**
     * 
     * @param speed motor power to apply to intake
  *@param angle in radians
  *
  */
  public void load(double speed, double angle){
    intakeMotor.set(speed);
    tiltToAngle(angle);

  }

  /**
   * 
   * @param angle motor to apply to intake
   * 
   */
  public void tiltToAngle(double angle) {
    double motorPower = armPID.calculate(armEncoder.getAbsolutePosition(), angle);
    armMotor.set(motorPower);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
