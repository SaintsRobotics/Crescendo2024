// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AmpOuttakeCommand extends Command {
  private IntakeSubsystem m_intakeSubsystem;

  /** Creates a new intakeCommand. */
  public AmpOuttakeCommand(IntakeSubsystem subsystem) {
    m_intakeSubsystem = subsystem;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.outtakeAmp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
