package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TagAlignConstants;
import frc.robot.Constants.TagAlignConstants.AlignPosition;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToTagCommand extends Command {

	final DriveSubsystem m_subsystem;
	final Pose2d m_target; // Desired pose
	final PIDController m_PIDxy;
	final PIDController m_PIDomega; // Omega is used to denote robot heading in radians

	double m_consecCount; // counter of number of consecutive cycles within tolerance

	/**
	 * 
	 * @param subsystem Subsystem to require
	 * @param alignPos  Position to align to
	 */
	public AlignToTagCommand(DriveSubsystem subsystem, AlignPosition alignPos) {
		m_subsystem = subsystem;
		addRequirements(m_subsystem);

		final Pose2d redTarget = TagAlignConstants.kTargetPoses[alignPos.ordinal()];

		// Blue positions are determined by flipping the x position and angle across the
		// center. If this logic does not work, then hard coded values may be better
		// (and faster). Indexing can be done by adding a constant to the ordinal when
		// indexing the array. This will allows accessing the blue positions which
		// should be put in the array after the red ones.
		m_target = DriverStation.getAlliance().get().compareTo(Alliance.Red) == 0 ? redTarget
				: new Pose2d(new Translation2d( // Flip poses of alliance is blue
						TagAlignConstants.kFieldWidth - redTarget.getX(),
						redTarget.getY()),
						new Rotation2d(MathUtil.angleModulus(Math.PI - redTarget.getRotation().getRadians())));

		m_PIDxy = new PIDController(TagAlignConstants.kTagPIDkPxy, TagAlignConstants.kTagPIDkIxy,
				TagAlignConstants.kTagPIDkDxy); // A feed forward control might also be needed here to overcome static friction
		m_PIDomega = new PIDController(TagAlignConstants.kTagPIDkPomega, TagAlignConstants.kTagPIDkIomega,
				TagAlignConstants.kTagPIDkDomega);

		m_PIDomega.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {
		m_PIDxy.reset(); // If a command instance is never reused then these three lines won't be needed
		m_PIDomega.reset();
		m_consecCount = 0;
	}

	@Override
	public void execute() {
		Pose2d current = m_subsystem.getPose();

		double xSpeed = MathUtil.clamp(m_PIDxy.calculate(current.getX(), m_target.getX()),
				-0, 0); // TODO: change clamp parameters and units

		double ySpeed = MathUtil.clamp(
				m_PIDxy.calculate(current.getY(), m_target.getY()),
				-0, 0); // TODO: change clamp parameters and units

		double omegaSpeed = MathUtil.clamp(
				m_PIDomega.calculate(current.getRotation().getRadians(), m_target.getRotation().getRadians()),
				-0, 0); // TODO: change clamp parameters and units

		// Check if within tolerance
		if (Math.abs(current.getX() - m_target.getX()) < TagAlignConstants.kTolxy && // x within range
				Math.abs(current.getY() - m_target.getY()) < TagAlignConstants.kTolxy && // y within range
				Math.abs( // omega within range
						current.getRotation().getRadians() - m_target.getRotation().getRadians()) < TagAlignConstants.kTolomega) {
			m_consecCount++;
		} else
			m_consecCount = 0;

			m_subsystem.drive(xSpeed, ySpeed, omegaSpeed, true);
	}

	@Override
	public void end(boolean interrupted) {
		// TODO: log finish reason to NT (either reached, proximity, or interrupted)
		// This is also a good place to start an instant command to move the outake to
		// the correct position
	}

	@Override
	public boolean isFinished() {
		Pose2d current = m_subsystem.getPose();
		return Math.pow(current.getX() - m_target.getX(), 2) + Math.pow(current.getY() - m_target.getY(), 2) > Math
				.pow(TagAlignConstants.kMinProx, 2) || // Exceed proximity.
				m_consecCount >= TagAlignConstants.kConsectol; // Reached setpoint
	}
}