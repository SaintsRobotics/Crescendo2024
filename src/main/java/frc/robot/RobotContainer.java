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
import frc.robot.Constants.IntakeConstants;
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

  private XboxController m_controller = new XboxController(0);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_intakeSubsystem.setDefaultCommand(m_controller.getLeftTriggerAxis() > 0.5
        ? new InstantCommand(m_intakeSubsystem::intakeDisk, m_intakeSubsystem)
        : new InstantCommand(m_intakeSubsystem::stopIntaking, m_intakeSubsystem));
  }

  public void periodic() {
    if (m_controller.getAButton()) {
      lastAButton = true;
      if (!lastAButton)
        IntakeDropped = !IntakeDropped;
    } else {
      lastAButton = false;
    }
  }

  private void configureBindings() {
    new JoystickButton(m_controller, Button.kY.value)
        .onTrue(new InstantCommand(() -> m_intakeSubsystem.tiltToAngle(IntakeConstants.kIntakeLoweredAngle), m_intakeSubsystem))
        .onFalse(new InstantCommand(m_intakeSubsystem::stopRotating, m_intakeSubsystem));
    new JoystickButton(m_controller, Button.kX.value)
        .onTrue(new InstantCommand(() -> m_intakeSubsystem.tiltToAngle(IntakeConstants.kIntakeRaisedAngle), m_intakeSubsystem))
        .onFalse(new InstantCommand(m_intakeSubsystem::stopRotating, m_intakeSubsystem));
    new JoystickButton(m_controller, Button.kB.value)
        .onTrue(new InstantCommand(() -> m_shooterSubsystem.spin(0.75), m_shooterSubsystem))
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.spin(0), m_shooterSubsystem));
    new JoystickButton(m_controller, Button.kA.value)
        .onTrue(new InstantCommand(() -> m_shooterSubsystem.spin(-0.75), m_shooterSubsystem))
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.spin(0), m_shooterSubsystem));
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