package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.TagAlignConstants;
import frc.robot.Constants.TagAlignConstants.AlignPosition;

public class AlignToTagCommand extends CommandBase {

	final Subsystem m_subsystem; // TODO: change type to drive subsystem
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
		Pose2d current = new Pose2d(); // TODO: get pose from april tag

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
		Pose2d current = new Pose2d(); // TODO: get pose from april tag
		return Math.pow(current.getX() - m_target.getX(), 2) + Math.pow(current.getY() - m_target.getY(), 2) > Math
				.pow(TagAlignConstants.kMinProx, 2) || // Exceed proximity
				m_consecCount >= TagAlignConstants.kConsectol; // Reached setpoint
	}
}