// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class SwerveModule {
  private final SparkFlex m_driveMotor;
  private final SparkFlex m_turningMotor;

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

    SparkFlexConfig driveConfig = new SparkFlexConfig();
    driveConfig.inverted(driveMotorReversed);
    driveConfig.idleMode(IdleMode.kBrake);

    m_driveMotor = new SparkFlex(driveMotorPort, MotorType.kBrushless);
    m_driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig turnConfig = new SparkFlexConfig();
    turnConfig.inverted(true);
    turnConfig.idleMode(IdleMode.kBrake);

    m_turningEncoder = new CANcoder(turningEncoderPort);
    m_turningMotor = new SparkFlex(turningMotorPort, MotorType.kBrushless);
    m_turningMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

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
    desiredState.optimize(getTurnEncoderAngle());
    m_state = desiredState;
    
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
