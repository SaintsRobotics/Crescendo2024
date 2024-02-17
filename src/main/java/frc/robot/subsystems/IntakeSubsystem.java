// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  //TODO: add constants
  CANSparkFlex intakeMotor = new CANSparkFlex(0, MotorType.kBrushless);
  CANSparkFlex armMotor = new CANSparkFlex(0, MotorType.kBrushless);
  

  PIDController intakeVeloPID = new PIDController(0,0,0);
  PIDController armPID = new PIDController(0,0,0);
  DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
  




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
  *@param angle in radians
  *@everyone join vc we are playing gartic phone
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
