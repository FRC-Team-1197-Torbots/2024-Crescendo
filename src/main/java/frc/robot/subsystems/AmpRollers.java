package frc.robot.subsystems;

import static frc.robot.Constants.AmpRollerConstants.RollerMotor;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.Constants.ShooterConstants;

public class AmpRollers extends SubsystemBase{

    private CANSparkMax m_RollerMotor;
    private PIDController m_PidController;
    private double targetRPM = 0;
    private double kP;

    public AmpRollers() {
        m_RollerMotor = new CANSparkMax(AmpRollerConstants.RollerMotor, MotorType.kBrushless);
        m_PidController = new PIDController(kP, AmpRollerConstants.kI, AmpRollerConstants.kD);
        SmartDashboard.putNumber("target rpm", targetRPM);
        SmartDashboard.putNumber("kP", kP);
    }

    public void setVoltage(double voltage) {
        m_RollerMotor.setVoltage(voltage);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    private double getPIDOutput() {
        double feedForward = (double)targetRPM / AmpRollerConstants.NEOMaxSpeed * ShooterConstants.NominalBatteryVoltage;
        double pidOutput = m_PidController.calculate(targetRPM - m_RollerMotor.getEncoder().getVelocity());
        return feedForward + pidOutput;
    }

    public void rileyAndKaiden() {
        targetRPM = SmartDashboard.getNumber("target rpm",targetRPM);
        kP = SmartDashboard.getNumber("kP", kP);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());

        // SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Roller RPM", m_RollerMotor.getEncoder().getVelocity());
       
        setVoltage(getPIDOutput());
    }
    

   
}
