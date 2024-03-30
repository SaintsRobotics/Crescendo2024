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

  private boolean deployed = false;
  private Timer m_timer = new Timer();

  private boolean m_compressorEnabled;

  private Value m_state;

  public ClimberSubsystem() {
    m_pHub = new PneumaticHub(2);

    solenoidOff();

    m_compressorEnabled = false;

    m_timer.start();
  }

  // Runs once every tick (~20ms)
  public void periodic() {
    m_leftSolenoid.set(m_state);
    m_rightSolenoid.set(m_state);
    SmartDashboard.putString("pneumatics state", m_state.name());
    SmartDashboard.putNumber("pressure", m_pHub.getPressure(0));
    SmartDashboard.putBoolean("Compressor Enabled", m_compressorEnabled);
    SmartDashboard.putBoolean("Compressor Running", m_pHub.getCompressor());

    if (m_state == Value.kForward){
      deployed = true;
    }
    if (m_state == Value.kReverse){
      m_timer.reset();
      if (m_timer.get() > 3.0){
        deployed = false;
      }
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
  }

  /**
   * Retracts both arms
   */
  public void reverse() {
    m_state = kReverse;
  }

  public Value getState(){
    return m_state;
  }

  public boolean getDeployed(){
    return deployed;
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
