package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
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

  private double togglecount = 0;
  private Timer m_timer = new Timer();

  public ClimberSubsystem() {
    m_pHub = new PneumaticHub(2);

    solenoidOff();

    m_compressorEnabled = false;
  }

  // Runs once every tick (~20ms)
  public void periodic() {
    m_leftSolenoid.set(m_state);
    m_rightSolenoid.set(m_state);
    SmartDashboard.putString("pneumatics state", m_state.name());
    SmartDashboard.putNumber("pressure", m_pHub.getPressure(0));
    SmartDashboard.putBoolean("Compressor Enabled", m_compressorEnabled);
    SmartDashboard.putBoolean("Compressor Running", m_pHub.getCompressor());

    if (togglecount >= 3){
      togglecount = 0;
    }
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
    togglecount++;
  }

  /**
   * Retracts both arms
   */
  public void reverse() {
    m_state = kReverse;
    togglecount++;
  }

  public Value getState(){
    return m_state;
  }

  public boolean deployed(){
    return togglecount > 0;
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
