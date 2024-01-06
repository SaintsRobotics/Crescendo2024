package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TagAlignConstants;
import frc.robot.Constants.TagAlignConstants.AlignPosition;

public class AlignToTagCommand extends CommandBase {

	final Subsystem m_subsystem; // TODO: change type to drive subsystem type
	final Pose2d m_target;
	final PIDController m_PIDxy;
	final PIDController m_PIDomega;

	double m_consecCount; // counter of number of consecutive cycles within tolerance

	/**
	 * 
	 * @param subsystem Subsystem to require. Subsystem must implement a drive
	 *                  method and a getPosition method.
	 * @param alignPos  Position to align to
	 */
	public AlignToTagCommand(Subsystem subsystem, AlignPosition alignPos) {
		m_subsystem = subsystem;
		addRequirements(m_subsystem);
		m_target = TagAlignConstants.kTargetPoses[alignPos.ordinal()];
		m_PIDxy = new PIDController(TagAlignConstants.kTagPIDkPxy, TagAlignConstants.kTagPIDkIxy,
				TagAlignConstants.kTagPIDkDxy);
		m_PIDomega = new PIDController(TagAlignConstants.kTagPIDkPomega, TagAlignConstants.kTagPIDkIomega,
				TagAlignConstants.kTagPIDkDomega);
		m_PIDomega.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {
		m_PIDxy.reset();
		m_PIDomega.reset();
		m_consecCount = 0;
	}

	@Override
	public void execute() {
		Pose2d current = poseFromAprilTag();

		double xSpeed = MathUtil.clamp(m_PIDxy.calculate(current.getX(), m_target.getX()),
				-0, 0); // TODO: change clamp parameters and units

		double ySpeed = MathUtil.clamp(
				m_PIDxy.calculate(current.getY(), m_target.getY()),
				-0, 0);

		double omegaSpeed = MathUtil.clamp(
				m_PIDomega.calculate(current.getRotation().getRadians(), m_target.getRotation().getRadians()),
				-0, 0);

		// Check if within tolerance
		if (Math.abs(current.getX() - m_target.getX()) < TagAlignConstants.kTolxy && // x within range
				Math.abs(current.getY() - m_target.getY()) < TagAlignConstants.kTolxy && // y within range
				Math.abs( // omega within range
						current.getRotation().getRadians() - m_target.getRotation().getRadians()) < TagAlignConstants.kTolomega) {
			m_consecCount++;
		} else
			m_consecCount = 0;

		// TODO: call drive function
	}

	@Override
	public void end(boolean interrupted) {
		// TODO: log finish reason to NT (either reached, proximity, or interrupted)
	}

	@Override
	public boolean isFinished() {
		Pose2d current = poseFromAprilTag();
		return Math.pow(current.getX() - m_target.getX(), 2) + Math.pow(current.getY() - m_target.getY(), 2) > Math
				.pow(TagAlignConstants.kMinProx, 2) || // Exceed proximity
				m_consecCount >= TagAlignConstants.kConsectol; // Reached setpoint
	}

	/**
	 * Gets pose from vision measurements from limelight
	 * 
	 * @return Robot Pose
	 */
	Pose2d poseFromAprilTag() { // TODO: move this to drive subsystem
		NetworkTable tb = NetworkTableInstance.getDefault().getTable("limelight");
		DoubleArrayEntry botpose = null; // index 0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw, 6: timestamp. all angles
																			// are in degrees

		try {
			botpose = tb.getDoubleArrayTopic("botpose").getEntry(new double[0]);
			double[] bp = botpose.getAtomic().value; // Not sure how threading works in WPIlib w/ command scheduler. If it is
																								// thread safe then atomic wont be needed. There is no need to use the
																								// atomic timestamp since the topic has the timestamp stored in it (7th
																								// element)

			if (botpose.exists()) {
				Pose2d visionPose = new Pose2d(new Translation2d(bp[0], bp[1]), new Rotation2d(bp[5] * Math.PI / 180));
				DriveConstants.kPoseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp() - (bp[6] / 1000));
			} else {
				// TODO: log limelight failure
			}

		} finally {
			if (botpose != null && botpose.isValid())
				botpose.close(); // TODO: verify this logic: make sure it will never throw an exception,
													// otherwise add another trycatch block
		}

		return DriveConstants.kPoseEstimator.getEstimatedPosition();
	}
}