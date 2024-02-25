// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmPosition;

public class IntakeArmPositionCommand extends Command {
  private IntakeSubsystem m_intakeSubsystem;

  private ArmPosition m_armPosition;

  /** Creates a new intakeCommand. */
  public IntakeArmPositionCommand(IntakeSubsystem subsystem, ArmPosition position) {
    m_intakeSubsystem = subsystem;
    m_armPosition = position;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setArmPosition(m_armPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.armAtSetpoint();
  }
}
