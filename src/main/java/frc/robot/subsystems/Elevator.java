package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
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
    private double error, elevatorVoltage, kp, ki, kd;
    private CANSparkMax m_Motor;
    private PIDController m_PidController;
    // private Encoder m_Encoder;
    private AbsoluteEncoder m_Encoder;
    
    private double marginOfError = 10;

    public Elevator() {
        m_Encoder = m_Motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        
        m_Motor = new CANSparkMax(ElevatorConstants.Motor, MotorType.kBrushless);
        m_PidController = new PIDController(kp, ki, kd);
        SmartDashboard.putNumber("Elevator kp", kp);
        SmartDashboard.putNumber("Elevator ki", ki);
        SmartDashboard.putNumber("Elevator kd", kd);
    }

    public void setVoltage(double voltage) {
        m_Motor.setVoltage(voltage);
    }

    public void setTargetPos(double value) {
        targetPos = value;
    }

    public void updateFromSmartDashboard() {
        kp = SmartDashboard.getNumber("Elevator kp", kp);
        ki = SmartDashboard.getNumber("Elevator ki", ki);
        kd = SmartDashboard.getNumber("Elevator kd", kd);
        m_PidController.setP(kp); 
        m_PidController.setI(ki); 
        m_PidController.setD(kd);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", m_Encoder.getPosition());
        runPID();
    }


    
    public void runPID() {
        error = targetPos - m_Encoder.getPosition(); // might need to switch negative
        elevatorVoltage = m_PidController.calculate(error);
        setVoltage(elevatorVoltage);
    }

    public boolean atAmpHeight() {
        return Math.abs(error) < marginOfError;
    }
   
}
