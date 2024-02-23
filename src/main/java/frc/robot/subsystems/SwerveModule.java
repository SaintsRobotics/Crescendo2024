// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class SwerveModule {
  private final CANSparkFlex m_driveMotor;
  private final CANSparkFlex m_turningMotor;

  private final CANcoder m_turningEncoder;

  private final PIDController m_turningPIDController = new PIDController(DriveConstants.kPModuleTurningController, 0,
      0);

  private SwerveModuleState m_state = new SwerveModuleState();
  private double m_distance;

  public double driveOutput;
  public double turnOutput;

  /**
   * Constructs a {@link SwerveModule}.
   *
   * @param driveMotorPort     The port of the drive motor.
   * @param turningMotorPort   The port of the turning motor.
   * @param turningEncoderPort The port of the turning encoder.
   * @param driveMotorReversed Whether the drive motor is reversed.
   */
  public SwerveModule(
      int driveMotorPort,
      int turningMotorPort,
      int turningEncoderPort,
      boolean driveMotorReversed) {
    m_driveMotor = new CANSparkFlex(driveMotorPort, MotorType.kBrushless);
    m_turningMotor = new CANSparkFlex(turningMotorPort, MotorType.kBrushless);
    m_turningEncoder = new CANcoder(turningEncoderPort);

    // converts default units to meters per second
    m_driveMotor.getEncoder().setVelocityConversionFactor(
        DriveConstants.kWheelDiameterMeters * Math.PI / 60 / DriveConstants.kDrivingGearRatio);
    m_driveMotor.getEncoder().setPositionConversionFactor(
        DriveConstants.kWheelDiameterMeters * Math.PI / DriveConstants.kDrivingGearRatio);

    m_driveMotor.setInverted(driveMotorReversed);

    m_driveMotor.setIdleMode(IdleMode.kBrake);

    m_turningMotor.setInverted(true);

    m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    m_distance += m_state.speedMetersPerSecond * Robot.kDefaultPeriod;

    // If the robot is real, then return the swerve module state by reading from the
    // actual encoders
    // If the robot is simulated, then return the swerve module state using the
    // expected values
    return Robot.isReal()
        ? new SwerveModulePosition(m_driveMotor.getEncoder().getPosition(),
            getTurnEncoderAngle())
        : new SwerveModulePosition(m_distance, m_state.angle);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    m_state = SwerveModuleState.optimize(desiredState, getTurnEncoderAngle());
    driveOutput = m_state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;

    turnOutput = m_turningPIDController.calculate(getTurnEncoderAngle().getRadians(),
        m_state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /**
   * Returns the angle of the turning CANcoder
   * 
   * The CANcoder now gives values in rotations which is useless, so this method
   * translates the CANcoder output into a Rotation2D
   * 
   * @return A Rotation2d of the absolute angle.
   */
  public Rotation2d getTurnEncoderAngle() {
    return new Rotation2d(m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
  }
}
