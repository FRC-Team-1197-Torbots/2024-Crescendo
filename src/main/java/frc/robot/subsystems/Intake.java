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

    private double testspeed = 0;
    private CANSparkMax MotorA;
    private DigitalInput m_BreakBeam;

    public Intake() {
        MotorA = new CANSparkMax(IntakeConstants.IntakeMotor, MotorType.kBrushless);
        MotorA.setIdleMode(IdleMode.kBrake);
        m_BreakBeam = new DigitalInput(IntakeConstants.breakBeam);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Break beam", gamePieceStored());
        // SmartDashboard.putNumber("Intake Outtake Speed", testspeed);
    }

    public void runIntake(double spd) {
        MotorA.set(-spd);
    }

    public void stopMotor() {
        MotorA.set(0);
    }

    public boolean gamePieceStored() {
        return !m_BreakBeam.get();
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
}