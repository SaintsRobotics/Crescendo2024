// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootSpeed;

public class ShooterSetSpeedCommand extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  double m_shooterSpeed = 0;


  double set_time = ShooterConstants.kShooterOnTime;
  Timer m_timer = new Timer();

  ShootSpeed m_shootSpeed;

  /** Creates a new ShootCommand. */
  public ShooterSetSpeedCommand(ShooterSubsystem shooterSubsystem, ShootSpeed shootSpeed, double time) {
    m_ShooterSubsystem = shooterSubsystem;
    addRequirements(m_ShooterSubsystem);

    set_time = time;

    m_shootSpeed = shootSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setShootingSpeed(m_shootSpeed);

    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > set_time;
  }
}
