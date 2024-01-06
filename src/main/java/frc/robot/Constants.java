package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

public final class Constants {
	public final static class TagAlignConstants {
		public final static double kTagPIDkPxy = 0;
		public final static double kTagPIDkIxy = 0;
		public final static double kTagPIDkDxy = 0;

		public final static double kTagPIDkPomega = 0;
		public final static double kTagPIDkIomega = 0;
		public final static double kTagPIDkDomega = 0;

		public static final Pose2d[] kTargetPoses = new Pose2d[] {}; // Poses are in the same order as the enumerator.
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
			Speaker
		}
	}

	public final static class DriveConstants {
		//TODO: init pose estimator with actual values
		public static final SwerveDrivePoseEstimator kPoseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
	}
}