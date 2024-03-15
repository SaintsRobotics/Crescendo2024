// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;;

public class SetLED extends Command {
  private LEDSubsystem m_ledSubsystem;

  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private Pose2d m_odometry = m_driveSubsystem.getOdometry();

  private double[] m_pointsX = { ShooterConstants.kTopShootX, ShooterConstants.kMiddleShootX,
      ShooterConstants.kBottomShootX, 
      //Intake Positions
      IntakeConstants.KLeftRingX,
      IntakeConstants.KRightRingX,
    };
  private double[] m_pointsY = { ShooterConstants.kTopShootY, ShooterConstants.kMiddleShootY,
      ShooterConstants.kBottomShootY, IntakeConstants.KTopLeftRingY,
      //Intake Positions
      IntakeConstants.KTopLeftRingY,
      IntakeConstants.KMiddleLeftRingY,
      IntakeConstants.KBottomLeftRingY,
      IntakeConstants.KTopRightRingY,
      IntakeConstants.KSecondTopRightRingY,
      IntakeConstants.KMiddleRightRingY,
      IntakeConstants.KSecondBottomRightRingY,
      IntakeConstants.KBottomRightRingY
    };
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
    double robotX = m_odometry.getX();
    double robotY = m_odometry.getY();
    for (int i = 0; i < 3; i++) {
      boolean validX = false;
      boolean validY = false;
      boolean validA = false;
      double X = m_pointsX[i];
      double Y = m_pointsY[i];
      double A = m_angles[i];
      if (X + ShooterConstants.kPositionTolerance > robotX
          && robotX > X - ShooterConstants.kPositionTolerance) {
        validX = true;
      }
      if (Y + ShooterConstants.kPositionTolerance > robotY
          && robotY > Y - ShooterConstants.kPositionTolerance) {
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

    //Lighting for Intake Positions
    //Run through the next 8 positions on the list
    for(int i = 3; i < 11; i++){
      //Figures out the rings x value (there is only 2 independent values per side)
      double X;
      if(i <= 5){
        X = m_pointsX[3];
      }else{
        X = m_pointsX[4];
      }
      //Calculate distance to point
      double dist = Math.hypot(robotX -X,robotY - m_pointsY[i]);
      //Check if within an acceptable distance
      if(dist >= IntakeConstants.kIntakeDist - IntakeConstants.KIntakePositionTolerance &&
        dist <= IntakeConstants.kIntakeDist + IntakeConstants.KIntakePositionTolerance ){
        //Estimates the proper angle
        double estimateAngle = Math.toDegrees(Math.atan2(robotY - m_pointsY[i],robotX - X));
        //Checks if the angle is within an acceptable measure
        if(Math.abs(m_odometry.getRotation().getDegrees() - estimateAngle) <= IntakeConstants.KIntakeAngleTolerance){
          m_ledSubsystem.setLED(0,0, 255);
        }
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
