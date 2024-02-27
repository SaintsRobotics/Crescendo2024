// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.IntakeArmPositionCommand;
import frc.robot.commands.NoteIntakeCommand;
import frc.robot.commands.NoteOuttakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  private final XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(IOConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {

    AutoBuilder.configureHolonomic(m_robotDrive::getPose, m_robotDrive::resetOdometry,
        m_robotDrive::getChassisSpeeds,
        m_robotDrive::autonDrive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
            DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Math.hypot(DriveConstants.kTrackWidth, DriveConstants.kWheelBase), // Drive base radius in meters. Distance
                                                                               // from robot center to furthest module.
            new ReplanningConfig(true, true)),
        () -> false, m_robotDrive);

    m_visionSubsystem.addConsumer(m_robotDrive::addVisionMeasurement);

    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(
                    -m_driverController.getLeftY(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - m_driverController
                        .getLeftTriggerAxis()
                        * IOConstants.kSlowModeScalar),
                // * 0.8,
                MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - m_driverController
                        .getLeftTriggerAxis()
                        * IOConstants.kSlowModeScalar),
                // * 0.8,
                MathUtil.applyDeadband(
                    -m_driverController.getRightX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond
                    * (1 - m_driverController
                        .getLeftTriggerAxis()
                        * IOConstants.kSlowModeScalar)
                    / 2,
                !m_driverController.getRightBumper()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureBindings() {
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // new JoystickButton(m_driverController, Button.kA.value).whileTrue(
    //     AutoBuilder.pathfindToPose(new Pose2d(2.8, 5.5, new Rotation2d()), new PathConstraints(
    //         DriveConstants.kMaxSpeedMetersPerSecond - 1, 5, DriveConstants.kMaxAngularSpeedRadiansPerSecond - 1, 5)));

    new JoystickButton(m_operatorController, Button.kX.value)
        .onTrue(new InstantCommand(() -> m_shooterSubsystem.spin(0.75), m_shooterSubsystem))
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.spin(0), m_shooterSubsystem));
    new JoystickButton(m_operatorController, Button.kY.value)
        .onTrue(new InstantCommand(() -> m_shooterSubsystem.spin(-0.75), m_shooterSubsystem))
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.spin(0), m_shooterSubsystem));

    new JoystickButton(m_operatorController, Button.kA.value)
        .onTrue(new InstantCommand(() -> m_climberSubsystem.forward(), m_robotDrive));
    new JoystickButton(m_operatorController, Button.kB.value)
        .onTrue(new InstantCommand(() -> m_climberSubsystem.reverse(), m_robotDrive));

    new Trigger(() -> {
      return m_driverController.getLeftTriggerAxis() > 0.5;
    }).whileTrue(
        new SequentialCommandGroup(
            new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Extended),
            new NoteIntakeCommand(m_intakeSubsystem),
            new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Retracted)))
        .onFalse(new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Retracted));

    new Trigger(() -> {
      return m_driverController.getRightTriggerAxis() > 0.5;
    }).whileTrue(
        new SequentialCommandGroup(
            new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Amp),
            new NoteOuttakeCommand(m_intakeSubsystem)))
        .onFalse(new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Retracted));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("New New Path");

    var alliance = DriverStation.getAlliance();
    PathPlannerPath autonPath = path;
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      autonPath = autonPath.flipPath();
    }
    m_robotDrive.resetOdometry(autonPath.getPreviewStartingHolonomicPose());

    // return AutoBuilder.followPath(autonPath);
    return null;
  }
}
