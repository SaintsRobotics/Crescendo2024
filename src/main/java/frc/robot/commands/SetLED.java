// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants.ShooterConstants;;

public class SetLED extends Command {
  private LEDSubsystem m_ledSubsystem;

  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private Pose2d m_odometry = m_driveSubsystem.getOdometry();

  private double[] m_pointsX = { ShooterConstants.kTopShootX, ShooterConstants.kMiddleShootX,
      ShooterConstants.kBottomShootX };
  private double[] m_pointsY = { ShooterConstants.kTopShootY, ShooterConstants.kMiddleShootY,
      ShooterConstants.kBottomShootY };
  private double[] m_angles = { ShooterConstants.kTopShootAngle, ShooterConstants.kMiddleShootAngle,
      ShooterConstants.kBottomShootAngle };

  /** Creates a new SetLED. */
  public SetLED(LEDSubsystem LED) {
    m_ledSubsystem = LED;
    addRequirements(m_ledSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    for (int i = 0; i < 3; i++) {
      boolean validX = false;
      boolean validY = false;
      boolean validA = false;
      double X = m_pointsX[i];
      double Y = m_pointsY[i];
      double A = m_angles[i];
      if (X + ShooterConstants.kPositionTolerance > m_odometry.getX()
          && m_odometry.getX() > X - ShooterConstants.kPositionTolerance) {
        validX = true;
      }
      if (Y + ShooterConstants.kPositionTolerance > m_odometry.getY()
          && m_odometry.getY() > Y - ShooterConstants.kPositionTolerance) {
        validY = true;
        break;
      }
      if (A + ShooterConstants.kAngleTolerance > m_odometry.getRotation().getDegrees()
          && m_odometry.getRotation().getDegrees() > A - ShooterConstants.kAngleTolerance) {
        validA = true;
        break;
      }
      if (validX && validY && validA) {
        m_ledSubsystem.setLED(0, 255, 0);
        break;
      } else {
        m_ledSubsystem.setLED(255, 0, 0);
      }
    }

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
    return false;
  }
}
