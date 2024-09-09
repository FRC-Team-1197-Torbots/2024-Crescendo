package frc.robot.subsystems;

import static frc.robot.Constants.AmpRollerConstants.RollerMotor;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpRollerConstants;

public class AmpRollers extends SubsystemBase{

    private CANSparkMax m_RollerMotor;
    private DigitalInput m_BeamBreak;
    
    public AmpRollers() {
        m_BeamBreak = new DigitalInput(0);
        m_RollerMotor = new CANSparkMax(AmpRollerConstants.RollerMotor, MotorType.kBrushless);
    }

    public void setVoltage(double voltage) {
        m_RollerMotor.setVoltage(voltage);
    }

    public boolean gamePieceStored() {
        return m_BeamBreak.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Amp Beam Break", m_BeamBreak.get());
  
    }
    

   
}
