package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Constants {
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

	public final static class DriveConstants {
		//TODO: init pose estimator with actual values
		public static final SwerveDrivePoseEstimator kPoseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
	}
}