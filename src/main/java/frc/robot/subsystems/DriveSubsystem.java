// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningEncoderOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftDriveMotorReversed,
      DriveConstants.kRearLeftTurningEncoderOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningEncoderOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightDriveMotorReversed,
      DriveConstants.kRearRightTurningEncoderOffset);

  private final AHRS m_gyro = new AHRS();
  private double m_gyroAngle;

  private final Timer m_headingCorrectionTimer = new Timer();
  private final PIDController m_headingCorrectionPID = new PIDController(DriveConstants.kPHeadingCorrectionController,
      0, 0);
  private SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  };

  // TODO: Experiment with different std devs in the pose estimator
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(), m_swerveModulePositions, new Pose2d());

  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field", m_field);
    m_headingCorrectionTimer.restart();
    m_headingCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_swerveModulePositions = new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };

    m_poseEstimator.update(Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle), m_swerveModulePositions);

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());
    SmartDashboard.putNumber("odometryX", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("odometryY", m_poseEstimator.getEstimatedPosition().getY());

    // AdvantageScope Logging
    double[] logData = {
        m_frontLeft.getPosition().angle.getDegrees(), m_frontLeft.driveOutput,
        m_frontRight.getPosition().angle.getDegrees(), m_frontRight.driveOutput,
        m_rearLeft.getPosition().angle.getDegrees(), m_rearLeft.driveOutput,
        m_rearRight.getPosition().angle.getDegrees(), m_rearRight.driveOutput,
    };
    SmartDashboard.putNumberArray("AdvantageScope Swerve States", logData);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation      Angular rotation speed of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    // If we are rotating, reset the timer
    if (rotation != 0) {
      m_headingCorrectionTimer.reset();
    }

    /*
     * Heading correction helps maintain the same heading and
     * prevents rotational drive while our robot is translating
     * 
     * For heading correction we use a timer to ensure that we
     * lose all rotational momentum before saving the heading
     * that we want to maintain
     */

    // TODO: Test heading correction without timer
    // TODO: Test heading correction using gyro's rotational velocity (if it is 0
    // then set heading instead of timer)

    // Save our desired rotation to a variable we can add our heading correction
    // adjustments to
    double calculatedRotation = rotation;

    double currentAngle = MathUtil.angleModulus(m_gyro.getRotation2d().getRadians());

    // If we are not translating or if not enough time has passed since the last
    // time we rotated
    if ((xSpeed == 0 && ySpeed == 0)
        || m_headingCorrectionTimer.get() < DriveConstants.kHeadingCorrectionTurningStopTime) {
      // Update our desired angle
      m_headingCorrectionPID.setSetpoint(currentAngle);
    } else {
      // If we are translating or if we have not rotated for a long enough time
      // then maintain our desired angle
      calculatedRotation = m_headingCorrectionPID.calculate(currentAngle);
    }

    // Depending on whether the robot is being driven in field relative, calculate
    // the desired states for each of the modules
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, calculatedRotation,
                Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle))
            : new ChassisSpeeds(xSpeed, ySpeed, calculatedRotation));

    setModuleStates(swerveModuleStates);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyroAngle = 0;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);

    // AdvantageScope Logging
    double[] logData = {
        desiredStates[0].angle.getDegrees(), desiredStates[0].speedMetersPerSecond,
        desiredStates[1].angle.getDegrees(), desiredStates[1].speedMetersPerSecond,
        desiredStates[2].angle.getDegrees(), desiredStates[2].speedMetersPerSecond,
        desiredStates[3].angle.getDegrees(), desiredStates[3].speedMetersPerSecond,
    };
    SmartDashboard.putNumberArray("AdvantageScope Swerve Desired States", logData);

    // Takes the integral of the rotation speed to find the current angle for the
    // simulator
    m_gyroAngle += DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond
        * Robot.kDefaultPeriod;
  }

}
