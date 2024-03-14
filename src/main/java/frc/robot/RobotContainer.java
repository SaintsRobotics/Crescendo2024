// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeArmPositionCommand;
import frc.robot.commands.NoteIntakeCommand;
import frc.robot.commands.NoteOuttakeCommand;
import frc.robot.commands.ShooterSetSpeedCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootSpeed;
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
  // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  private final XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(IOConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Shoot",
        new SequentialCommandGroup(
            new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Shooting, ShooterConstants.kShooterOnTime),
            new ParallelDeadlineGroup(new WaitCommand(0.8), new NoteOuttakeCommand(m_intakeSubsystem)),
            new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Off, ShooterConstants.kShooterOffTime)));

    NamedCommands.registerCommand("Intake",
        new SequentialCommandGroup(
            new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Extended),
            new NoteIntakeCommand(m_intakeSubsystem),
            new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Retracted)));

    NamedCommands.registerCommand("Pre-Speed - 30%",
      new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Halfway, 0.01));

    NamedCommands.registerCommand("Intake in",
        new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Retracted));

    AutoBuilder.configureHolonomic(m_robotDrive::getPose, m_robotDrive::resetOdometry,
        m_robotDrive::getChassisSpeeds,
        m_robotDrive::autonDrive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
            DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Math.hypot(DriveConstants.kTrackWidth, DriveConstants.kWheelBase), // Drive base radius in
                                                                               // meters. Distance
                                                                               // from robot center to
                                                                               // furthest module.
            new ReplanningConfig(false, false)),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, m_robotDrive);

    // new SequentialCommandGroup(new ShooterSetSpeedCommand(m_shooterSubsystem,
    // ShootSpeed.Shooting),
    // new ParallelDeadlineGroup(new WaitCommand(0.5), new
    // NoteOuttakeCommand(m_intakeSubsystem))));

    m_visionSubsystem.addConsumer(m_robotDrive::addVisionMeasurement);
    m_intakeSubsystem.addConsumer(m_ledSubsystem::addIntakeStatus);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
                        .getRightTriggerAxis()
                        * IOConstants.kSlowModeScalar),
                // * 0.8,
                MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - m_driverController
                        .getRightTriggerAxis()
                        * IOConstants.kSlowModeScalar),
                // * 0.8,
                MathUtil.applyDeadband(
                    -m_driverController.getRightX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond
                    * (1 - m_driverController
                        .getRightTriggerAxis()
                        * IOConstants.kSlowModeScalar)
                    / 2,
                !m_driverController.getLeftBumper()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureBindings() {
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // new JoystickButton(m_driverController, Button.kA.value).whileTrue(
    // AutoBuilder.pathfindToPose(new Pose2d(2.8, 5.5, new Rotation2d()), new
    // PathConstraints(
    // DriveConstants.kMaxSpeedMetersPerSecond - 1, 5,
    // DriveConstants.kMaxAngularSpeedRadiansPerSecond - 1, 5)));

    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(new SequentialCommandGroup(new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Shooting, ShooterConstants.kShooterOnTime),
            new NoteOuttakeCommand(m_intakeSubsystem)))
        .onFalse(new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Off, ShooterConstants.kShooterOffTime));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Amp))
        .onFalse(new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Retracted));

    // Intake, Driver Controller Right Trigger
    new Trigger(() -> {
      return m_driverController.getLeftTriggerAxis() > 0.5;
    }).whileTrue(
        new SequentialCommandGroup(
            new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Extended),
            new NoteIntakeCommand(m_intakeSubsystem),
            new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Retracted)))
        .onFalse(new IntakeArmPositionCommand(m_intakeSubsystem, ArmPosition.Retracted));

    // Outtake, Operator Controller Right Trigger
    new Trigger(() -> {
      return m_operatorController.getRightTriggerAxis() > 0.5;
    }).whileTrue(new NoteOuttakeCommand(m_intakeSubsystem));

    new Trigger(() -> {
      return m_operatorController.getLeftTriggerAxis() > 0.5;
    }).onTrue(new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Shooting, ShooterConstants.kShooterOnTime))
        .onFalse(new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Off, ShooterConstants.kShooterOffTime));

    // Climber Up, Operator Controller Right Bumper + A Button
    // new Trigger(() -> {
    //   return m_operatorController.getAButton() && m_operatorController.getRightBumper();
    // }).whileTrue(new InstantCommand(() -> m_climberSubsystem.forward()));

    // // Climber Down, Operator Controller Right Bumper + B Button
    // new Trigger(() -> {
    //   return m_operatorController.getBButton() && m_operatorController.getRightBumper();
    // }).whileTrue(new InstantCommand(() -> m_climberSubsystem.reverse()));

    // Toggle Distance Sensor, Operatoe Controller Left Bumper + Start Button
    new Trigger(() -> {
      return m_operatorController.getLeftBumper() && m_operatorController.getStartButton();
    }).onTrue(new InstantCommand(() -> m_intakeSubsystem.toggleDistanceSensor()));
  }

  /**
   * Reset all subsystems on teleop init
   */
  public void resetAllSubsystems() {
    m_intakeSubsystem.reset();
    m_shooterSubsystem.reset();
    m_robotDrive.reset();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    /// List<PathPlannerPath> pathGroup =
    /// PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());
    // PathPlannerAuto path = PathPlannerAuto.getPathGroupFromAutoFile(pathGroup);

    // var alliance = DriverStation.getAlliance();
    // PathPlannerPath pathGroup = path;
    // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
    // autonPath = autonPath.flipPath();
    // }
    // m_robotDrive.resetOdometry(autonPath.getPreviewStartingHolonomicPose());

    // return new PathPlannerAuto(autoChooser.getSelected().getName());
    return autoChooser.getSelected();

    // return new SequentialCommandGroup(
    //         new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Shooting, 3),
    //         new ParallelDeadlineGroup(new WaitCommand(1.5), new NoteOuttakeCommand(m_intakeSubsystem)),
    //         new ShooterSetSpeedCommand(m_shooterSubsystem, ShootSpeed.Off, 0.01)
    //         , new ParallelDeadlineGroup(new WaitCommand(20), new RepeatCommand(new InstantCommand(() -> m_robotDrive.autonDrive(new ChassisSpeeds(3, 0, 0)))))
    //         );



  }
}
