package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private double targetPos = ElevatorConstants.StorePos; 
    private double error, elevatorVoltage, kp = 0.9, kd;
    private double feedForward = -0.3;
    private CANSparkMax m_Motor;
    private PIDController m_PidController;
    // private Encoder m_Encoder;
    private RelativeEncoder m_Encoder;
    private BooleanSupplier m_BeamBreak;
    private double marginOfError = 0.4;

    public Elevator() {
        
        m_Motor = new CANSparkMax(ElevatorConstants.Motor, MotorType.kBrushless);
        m_Encoder = m_Motor.getEncoder();
        m_PidController = new PIDController(kp, 0, kd);
        SmartDashboard.putNumber("Elevator kp", kp);
        SmartDashboard.putNumber("Elevator kd", kd);
    }

    public void setVoltage(double voltage) {
        m_Motor.setVoltage(voltage);
    }

    public void setTargetPos(double value) {
        targetPos = value;
    }

    public void giveBeamBreak(BooleanSupplier beamBreak) {
        m_BeamBreak = beamBreak;
    }

    public boolean gamePieceStored() {
        return m_BeamBreak.getAsBoolean();
    }

    public void updateFromSmartDashboard() {
        kp = SmartDashboard.getNumber("Elevator kp", kp);
        kd = SmartDashboard.getNumber("Elevator kd", kd);
        m_PidController.setP(kp); 
        m_PidController.setD(kd);
    }

    public double getPosition() {
        return -m_Encoder.getPosition();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", getPosition());
        runPID();
    }


    
    public void runPID() {
        error = targetPos - getPosition(); // might need to switch negative
        elevatorVoltage = m_PidController.calculate(error);
        SmartDashboard.putNumber("elevator voltage", elevatorVoltage);
        setVoltage(feedForward + elevatorVoltage);
    }

    public boolean atAmpHeight() {
        return Math.abs(error) < marginOfError;
    }
   
}
