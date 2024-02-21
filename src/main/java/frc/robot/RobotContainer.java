// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.IntakeSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public void periodic() {
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureBindings() {
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue( new IntakeCommand(m_intakeSubsystem) );
    // TODO: Move shoot commands to operator controller
    // new JoystickButton(m_driverController, Button.kY.value)
    //     .onTrue(new InstantCommand(() -> m_intakeSubsystem.tiltToAngle(IntakeConstants.kIntakeLoweredAngle),
    //         m_intakeSubsystem))
    //     .onFalse(new InstantCommand(m_intakeSubsystem::stopRotating, m_intakeSubsystem));
    // new JoystickButton(m_driverController, Button.kX.value)
    //     .onTrue(new InstantCommand(() -> m_intakeSubsystem.tiltToAngle(IntakeConstants.kIntakeRaisedAngle),
    //         m_intakeSubsystem))
    //     .onFalse(new InstantCommand(m_intakeSubsystem::stopRotating, m_intakeSubsystem));

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