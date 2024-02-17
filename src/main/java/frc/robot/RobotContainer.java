// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private boolean IntakeDropped = false;
  private boolean lastAButton = false;

  private XboxController controller = new XboxController(0);
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_intakeSubsystem.setDefaultCommand(new InstantCommand(()-> m_IntakeSubsystem.load(controller.getRightTriggerAxis()-controller.getLeftTriggerAxis(),IntakeDropped ? IntakeConstants.IntakeDroppedAngle : IntakeConstants.IntakeRaisedAngle),m_IntakeSubsystem));
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}