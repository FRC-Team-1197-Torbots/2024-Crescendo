package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpRollerConstants;

public class AmpRollers extends SubsystemBase {

    private CANSparkMax m_RollerMotor;
    private DigitalInput m_BeamBreak;
    
    public AmpRollers() {
        SmartDashboard.putBoolean("Override Amp Beam Brake", false);
        m_BeamBreak = new DigitalInput(AmpRollerConstants.BeamBreakChannel);
        m_RollerMotor = new CANSparkMax(AmpRollerConstants.RollerMotor, MotorType.kBrushless);
    }

    public void setVoltage(double voltage) {
        m_RollerMotor.setVoltage(voltage);
    }

    public boolean gamePieceStored() {
        if(SmartDashboard.getBoolean("Override Amp Beam Brake", false))  {
            return false;
        }
        return m_BeamBreak.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Game Piece Stored Amp", m_BeamBreak.get());
    }
}
