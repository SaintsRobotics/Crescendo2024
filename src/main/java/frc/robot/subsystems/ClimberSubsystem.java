package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_leftSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH,
      ClimberConstants.leftForwardChannel, ClimberConstants.leftReverseChannel);
  private final DoubleSolenoid m_rightSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH,
      ClimberConstants.rightReverseChannel, ClimberConstants.rightForwardChannel);
  private PneumaticHub m_pHub;

  private boolean m_compressorEnabled;

  private Value m_state;

  public ClimberSubsystem() {
    m_compressorEnabled = false;
    m_pHub = new PneumaticHub(2);

    solenoidOff();

    m_compressorEnabled = false;
    toggleCompressor();
  }

  // Runs once every tick (~20ms)
  public void periodic() {
    m_leftSolenoid.set(m_state);
    m_rightSolenoid.set(m_state);
    SmartDashboard.putString("pneumatics state", m_state.name());
    SmartDashboard.putNumber("pressure", m_pHub.getPressure(0));
  }

  /**
   * Sets the state of the solenoid to off
   */
  public void solenoidOff() {
    m_state = kOff;
  }

  /**
   * Extends both arms
   */
  public void forward() {
    m_state = kForward;
  }

  /**
   * Retracts both arms
   */
  public void reverse() {
    m_state = kReverse;
  }

  /**
   * Toggles the state of the compressor (on/off)
   */
  public void toggleCompressor() {
    m_compressorEnabled = !m_compressorEnabled;
    if (m_compressorEnabled) {
      m_pHub.enableCompressorAnalog(ClimberConstants.minPressure, ClimberConstants.maxPressure);
    } else {
      m_pHub.disableCompressor();
    }
  }
}
