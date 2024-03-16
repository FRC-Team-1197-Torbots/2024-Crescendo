package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private double testspeed = 4.2;
    private CANSparkMax MotorA;
    private DigitalInput m_BreakBeam;
    private DigitalInput m_BreakBeam2;
    public boolean finishedIntaking;
    public Intake() {
        MotorA = new CANSparkMax(IntakeConstants.IntakeMotor, MotorType.kBrushless);
        MotorA.setIdleMode(IdleMode.kBrake);
        m_BreakBeam = new DigitalInput(IntakeConstants.breakBeam);
        m_BreakBeam2 = new DigitalInput(IntakeConstants.breakBeam2);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Finished Intaking state", finishedIntakeState());
        // SmartDashboard.putBoolean("Break beam", gamePieceStored());
        SmartDashboard.putBoolean("Break beam1", !m_BreakBeam.get());
        SmartDashboard.putBoolean("Break beam2", m_BreakBeam2.get());
        SmartDashboard.putBoolean("Intake Spinning?", intakeMoving());
        SmartDashboard.putNumber("Intake Velocity", MotorA.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Intake Outtake Speed", testspeed);
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
        return !m_BreakBeam.get() || m_BreakBeam2.get();
        // return false;
    }

    public void setMotorMode(IdleMode mode) {
        MotorA.setIdleMode(mode);
    }

    public void testIntake() {
        MotorA.set(testspeed);
    }

    public void incrementIntake(double amount) {
        testspeed += amount;
    }

    public boolean intakeMoving(){
        return MotorA.getEncoder().getVelocity() >= 200;
    }
}