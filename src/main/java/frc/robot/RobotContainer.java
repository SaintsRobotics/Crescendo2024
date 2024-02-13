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

public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final climberSubSystem climberSubSystem = new climberSubSystem();
  public RobotContainer() {
    configureBindings();
    
    


  }

  private void configureBindings() {
    //When Y button is pressed, the climber will toggle up or down
    //based on current position, it will be reversed
    new JoystickButton(m_controller, Button.kY.value)
    .onTrue(new InstantCommand(climberSubSystem::toggle));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
