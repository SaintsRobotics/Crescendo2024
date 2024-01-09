package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
  public static class IOConstants {
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

  public final static class TagAlignConstants {
		public final static double kTagPIDkPxy = 0; // xy PID constants
		public final static double kTagPIDkIxy = 0;
		public final static double kTagPIDkDxy = 0;

		public final static double kTagPIDkPomega = 0; // omega PID constants
		public final static double kTagPIDkIomega = 0;
		public final static double kTagPIDkDomega = 0;

		public static final Pose2d[] kTargetPoses = new Pose2d[] {
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), //Amp
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), //SourceLeft
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), //etc.
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)) //StageCenterRight
		}; // Poses are in the same order as the enumerator.
		// TODO: populate poses

		//TODO: tune all these parameters
		public static final double kTolxy = 0.1; // tolerance in meters for x and y
		public static final double kTolomega = 0.1; // tolerance in radians for omega
		public static final double kConsectol = 10; // onsecutive cycles of being within tolerance needed to end command
		public static final double kMinProx = 1; // minimum proximity radius in meters.
		// To avoid accidents, the robot must be
		// within the minimum proximity for the tag alignment command to run.
		// If the robot exceeds the minimum proximity the command will finish

		public static enum AlignPosition { // DON'T CHANGE THE ORDER OF THESE. Add new items to the end
			Amp,
			SourceLeft,
			SourceCenter,
			SourceRight,
			Speaker,
			StageLeftLeft, // The left side of the left stage
			StageLeftCenter, // the center side of the left stage
			StageLeftRight, // etc.
			StageRightLeft,
			StageRightCenter,
			StageRightRight,
			StageCenterLeft,
			StageCenterCenter,
			StageCenterRight // the right side of the center stage
		} //stage left, stage right, and stage center are the locations written in the game manual pg. 28

		public static final double kFieldWidth = 651.25 * 0.0254; // Width of the field in meters

		public static final NetworkTableInstance kNTInstance = NetworkTableInstance.create(); // Ideally all network table
																																													// instances should be put in
																																													// a wrapper and only the
																																													// default instance should be
																																													// used.
		// TODO: write unit tests that write to NT limelight and verify PID output
		// direction is correct. Needs to be implemented after drive since it uses pose
		// estimator.
	}
}
