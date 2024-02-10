package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

public class climberSubSystem {
    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, climberConst.climberForwardChannel, climberConst.climberReverseChannel);
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
    private boolean enableCompressor = true;
    

    public climberSubSystem(){
        m_doubleSolenoid.set(kOff);
        m_compressor.disable();
        m_compressor.enableAnalog(climberConst.minPressure, climberConst.maxPressure);
    }
    
    public void periodic() {
        SmartDashboard.putNumber("Pressure", m_compressor.getPressure());
        SmartDashboard.putBoolean("Compressor Enabled", m_compressor.isEnabled());
        SmartDashboard.putBoolean("Startup Compressor", enableCompressor);
    }

    public void off(){
        m_doubleSolenoid.set(kOff);
    }

    public void forward() {
        m_doubleSolenoid.set(kForward);
    }

    public void reverse() {
        m_doubleSolenoid.set(kReverse);
    }

    public void toggle() {
        m_doubleSolenoid.toggle();
    }

    public void toggleCompresor(){
        enableCompressor = !enableCompressor;
        if (enableCompressor){
            m_compressor.enableAnalog(climberConst.minPressure, climberConst.maxPressure);
        }
        else{
            m_compressor.disable();
        }
    }
}
