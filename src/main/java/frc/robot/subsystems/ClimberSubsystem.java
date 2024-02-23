package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ClimberConstants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

public class ClimberSubsystem {

    private final DoubleSolenoid m_leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.leftForwardChannel, ClimberConstants.leftReverseChannel);
    private final DoubleSolenoid m_rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.rightForwardChannel, ClimberConstants.rightReverseChannel);
    
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
    
    private boolean enableCompressor = true;
    
    public ClimberSubsystem(){
        m_leftSolenoid.set(kOff);
        m_rightSolenoid.set(kOff);
        m_compressor.disable();
        m_compressor.enableAnalog(ClimberConstants.minPressure, ClimberConstants.maxPressure);
    }
    
    //Runs once every tick (~20ms)
    public void periodic() {
        // SmartDashboard.putNumber("Pressure", m_compressor.getPressure());
        // SmartDashboard.putBoolean("Compressor Enabled", m_compressor.isEnabled());
        // SmartDashboard.putBoolean("Startup Compressor", enableCompressor);
    }
    /*
     * Sets the state of the solenoid to off 
     */
    public void off(){
        m_leftSolenoid.set(kOff);
        m_rightSolenoid.set(kOff);
    }

    //TODO: fix doc
    /*
     * Extends both arms
     */
    public void forward() {
        m_leftSolenoid.set(kForward);
        m_rightSolenoid.set(kForward);
    }

    //TODO:fix doc
    /*
     * Retracts the arm
     */
    public void reverse() {
        m_leftSolenoid.set(kReverse);
        m_rightSolenoid.set(kReverse);
    }
    /*
     * Toggles the state of the climber
     */

    public void toggle() {
        m_leftSolenoid.toggle();;
        m_rightSolenoid.toggle();;
    }

    //Toggles the state of the compressor (on/off)
    public void toggleCompresor(){
        enableCompressor = !enableCompressor;
        if (enableCompressor){
            m_compressor.enableAnalog(ClimberConstants.minPressure, ClimberConstants.maxPressure);
        }
        else{
            m_compressor.disable();
        }
    }
}
