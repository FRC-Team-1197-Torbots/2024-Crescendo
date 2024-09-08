package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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
    private Encoder m_Encoder;
    private double marginOfError = 10;
    // private Encoder m_Encoder;

    public Elevator() {
        m_Motor = new CANSparkMax(ElevatorConstants.Motor, MotorType.kBrushless);
        m_PidController = new PIDController(kp, ki, kd);
        m_Encoder = new Encoder(ElevatorConstants.encoderChannelA, ElevatorConstants.encoderChannelB);
    }

    public void setVoltage(double voltage) {
        m_Motor.setVoltage(voltage);
    }

    public void setTargetPos(double value) {
        targetPos = value;
    }


    @Override
    public void periodic() {
        SmartDashboard.getNumber("Elevator kp", kp);
        SmartDashboard.getNumber("Elevator ki", ki);
        SmartDashboard.getNumber("Elevator kd", kd);
        SmartDashboard.putNumber("Elevator Encoder", m_Encoder.get());
        runPID();
    }
    
    public void runPID() {
        error = targetPos - m_Encoder.get();
        elevatorVoltage = m_PidController.calculate(error);
        setVoltage(elevatorVoltage);
    }

    public boolean atAmpHeight() {
        return Math.abs(error) < marginOfError;
    }
   
}
