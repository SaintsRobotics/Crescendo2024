// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * Input/Output constants
   */
  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kControllerDeadband = 0.2;
    public static final double kSlowModeScalar = 0.8;
  }

  public static final class DriveConstants {
    // TODO: set motor and encoder constants
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kRearRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kRearLeftTurningMotorPort = 4;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kRearRightTurningMotorPort = 8;

    public static final int kFrontLeftTurningEncoderPort = 9;
    public static final int kRearLeftTurningEncoderPort = 10;
    public static final int kFrontRightTurningEncoderPort = 11;
    public static final int kRearRightTurningEncoderPort = 12;

    public static final double kFrontLeftTurningEncoderOffset = 0;
    public static final double kRearLeftTurningEncoderOffset = 0;
    public static final double kFrontRightTurningEncoderOffset = 0;
    public static final double kRearRightTurningEncoderOffset = 0;

    // TODO: Test motor orientations before driving on an actual robot
    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    /** Distance between centers of right and left wheels on robot (in meters). */
    public static final double kTrackWidth = 0.5;

    /** Distance between front and back wheels on robot (in meters). */
    public static final double kWheelBase = 0.5;

    /** Diameter of each wheel in the SDS MK4i swerve module (in meters) */
    public static final double kWheelDiameterMeters = 0.1;

    /** Gear ratio between the motor and the wheel. */
    public static final double kDrivingGearRatio = 8.14; // SDS MK4i's in L1 configuration

    // TODO: Tune this PID before running on a robot on the ground
    public static final double kPModuleTurningController = -0.3;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    /** For a a SDS Mk4i L1 swerve base with Neos */
    public static final double kMaxSpeedMetersPerSecond = 3.6576;
    /** For a a SDS Mk4i L1 swerve base with Neos */
    public static final double kMaxAngularSpeedRadiansPerSecond = 15.24 / 3;

    /** Heading Correction */
    public static final double kHeadingCorrectionTurningStopTime = 0.2;
    // TODO: Tune this PID before running on a robot on the ground
    public static final double kPHeadingCorrectionController = 5;
  }

  public static final class VisionConstants {
    // TODO: Update cam pose relative to center of bot
    public static final Pose3d kCamPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final double[] kLimelightCamPose = {
        kCamPose.getX(),
        kCamPose.getY(),
        kCamPose.getZ(),
        kCamPose.getRotation().getX(),
        kCamPose.getRotation().getY(),
        kCamPose.getRotation().getZ() };

    // TODO: Experiment with different std devs in the pose estimator
    public static final Vector<N3> kOdometrySTDDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Vector<N3> kVisionSTDDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    // Field size in meters
    public static final double kFieldWidth = 8.21055;
    public static final double kFieldLength = 16.54175;
  }

}
