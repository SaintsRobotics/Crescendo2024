package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

public class ClimberSubsystem extends SubsystemBase{

  private final DoubleSolenoid m_leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      ClimberConstants.leftForwardChannel, ClimberConstants.leftReverseChannel);
  private final DoubleSolenoid m_rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      ClimberConstants.rightForwardChannel, ClimberConstants.rightReverseChannel);

  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  private boolean m_compressorEnabled;

  private Value m_state;

  public ClimberSubsystem() {
    m_compressorEnabled = false;
    solenoidOff();
    toggleCompressor();
    }

  // Runs once every tick (~20ms)
  public void periodic() {
    m_leftSolenoid.set(m_state);
    m_rightSolenoid.set(m_state);
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
      m_compressor.enableAnalog(ClimberConstants.minPressure, ClimberConstants.maxPressure);
    } else {
      m_compressor.disable();
    }
  }
}
