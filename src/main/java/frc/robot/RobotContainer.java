// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private boolean IntakeDropped = false;
  private boolean lastAButton = false;

  private XboxController controller = new XboxController(0);
  public RobotContainer() {
    configureBindings();
    m_IntakeSubsystem.setDefaultCommand(new InstantCommand(()-> m_IntakeSubsystem.load(controller.getRightTriggerAxis()-controller.getLeftTriggerAxis(),IntakeDropped ? IntakeConstants.IntakeDroppedAngle : IntakeConstants.IntakeRaisedAngle),m_IntakeSubsystem));
  }

  public void periodic(){
    if(controller.getAButton()){
      lastAButton = true;
      if (!lastAButton)
        IntakeDropped = !IntakeDropped;
    }
    else {
      lastAButton = false;
    }
  }

  private void configureBindings() {
    new JoystickButton(controller, Button.kB.value)
      .onTrue(new InstantCommand(() -> m_ShooterSubsystem.spin(0.75), m_ShooterSubsystem))
      .onFalse(new InstantCommand(() -> m_ShooterSubsystem.spin(0), m_ShooterSubsystem));
    new JoystickButton(controller, Button.kA.value)
      .onTrue(new InstantCommand(() -> m_ShooterSubsystem.spin(-0.75), m_ShooterSubsystem))
      .onFalse(new InstantCommand(() -> m_ShooterSubsystem.spin(0), m_ShooterSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}