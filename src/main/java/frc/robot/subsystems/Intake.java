package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax MotorA;
    private DigitalInput m_BreakBeam;
    private DigitalInput m_BreakBeam2;
    public boolean finishedIntaking;

    public Intake() {
        MotorA = new CANSparkMax(IntakeConstants.IntakeMotor, MotorType.kBrushless);
        MotorA.setIdleMode(IdleMode.kBrake);
        m_BreakBeam = new DigitalInput(IntakeConstants.breakBeam);
        m_BreakBeam2 = new DigitalInput(IntakeConstants.breakBeam2);
        SmartDashboard.putBoolean("Override Beam Break 1", false);
        SmartDashboard.putBoolean("Override Beam Break 2", false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Break beam1", !m_BreakBeam.get());
        SmartDashboard.putBoolean("Break beam2", m_BreakBeam2.get());
        SmartDashboard.putBoolean("Game Piece Stored", gamePieceStored());
    }

    public void setIntakeVoltage(double voltage) {
        MotorA.setVoltage(-voltage);
    }

    public void runIntake(double spd) {
        MotorA.set(-spd);
    }

    public void stopMotor() {
        MotorA.set(0);
    }

    public void setFinishedIntake(boolean finished){
        finishedIntaking = finished;
    }
    
    public boolean finishedIntakeState(){
        return finishedIntaking;
    }

    public boolean gamePieceStored() {
        if(SmartDashboard.getBoolean("Override Beam Break 1",false))
            return m_BreakBeam2.get();
        if (SmartDashboard.getBoolean("Override Beam Break 2",false))
            return !m_BreakBeam.get();
        return !m_BreakBeam.get() || m_BreakBeam2.get();
    }

    public void setMotorMode(IdleMode mode) {
        MotorA.setIdleMode(mode);
    }

    public boolean intakeMoving(){
        return Math.abs(MotorA.getEncoder().getVelocity()) >= 200;
    }
}