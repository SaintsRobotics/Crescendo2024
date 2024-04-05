// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DefaultLEDCommand extends Command {
  private LEDSubsystem m_ledSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private ClimberSubsystem m_climberSubsystem;

  private double armSetpoint;
  private double shootSpeed;
  private int[] rgb = new int[3];

  /** Creates a new SetLED. */
  public DefaultLEDCommand(LEDSubsystem LED, IntakeSubsystem intake, ShooterSubsystem shooter, ClimberSubsystem climb) {
    m_ledSubsystem = LED;
    m_intakeSubsystem = intake;
    m_shooterSubsystem = shooter;
    m_climberSubsystem = climb;
    addRequirements(m_ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // default color is invalid so the subsystem will set it to rainbow
    rgb[0] = 256;
    rgb[1] = 256;
    rgb[2] = 256;

    //check for arm angle & haveNote
    armSetpoint = m_intakeSubsystem.getArmPosition();
    if (armSetpoint == IntakeConstants.kIntakeLoweredAngle) {
      rgb[0] = 255;
      rgb[1] = 0;
      rgb[2] = 0;
    } else if (armSetpoint == IntakeConstants.kIntakeRaisedAngle && m_intakeSubsystem.haveNote()) {
      rgb[0] = 0;
      rgb[1] = 0;
      rgb[2] = 255;
    }

    //check for shooter spinup and override b4 value
    shootSpeed = m_shooterSubsystem.returnCurrentSpeed();
    if (shootSpeed > 500 && shootSpeed < 1899 * Math.PI) { // if charging up
      rgb[0] = 128;
      rgb[1] = 0;
      rgb[2] = 255;
    } else if (shootSpeed > 1899 * Math.PI) { // if the shooter is ready to shoot
      rgb[0] = 0;
      rgb[1] = 255;
      rgb[2] = 0;
    }

    //check for pneumatics state and override b4 value
    if (m_climberSubsystem.getState() == Value.kForward) {
      rgb[0] = 257;
      rgb[1] = 257;
      rgb[2] = 257;
    }

    if (m_intakeSubsystem.getArmPosition() == IntakeConstants.kIntakeAmpScoringAngle) {
      if (m_intakeSubsystem.ampReady()) {
        rgb[0] = 0;
        rgb[1] = 255;
        rgb[2] = 0;   
      }
      else {
        rgb[0] = 0;
        rgb[1] = 255;
        rgb[2] = 0;
      }
    }


    m_ledSubsystem.setLED(rgb[0], rgb[1], rgb[2]);
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
    return true;
  }
}
