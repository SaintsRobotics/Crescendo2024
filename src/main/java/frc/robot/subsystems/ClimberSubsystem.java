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

  private boolean enableCompressor = true;

  private Value m_state;

  public ClimberSubsystem() {
        solenoidOff();
        m_compressor.disable();
        m_compressor.enableAnalog(ClimberConstants.minPressure, ClimberConstants.maxPressure);
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
  /*
   * Toggles the state of the climber
   */

  public void toggle() {
    if(m_state == kForward){
      m_state = kReverse;
    }else if(m_state == kReverse){
      m_state = kForward;
    }
  }

  // Toggles the state of the compressor (on/off)
  public void toggleCompresor() {
    enableCompressor = !enableCompressor;
    if (enableCompressor) {
      m_compressor.enableAnalog(ClimberConstants.minPressure, ClimberConstants.maxPressure);
    } else {
      m_compressor.disable();
    }
  }
}
