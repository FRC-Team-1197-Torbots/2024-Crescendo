package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{

    private CANSparkMax m_Motor;
    // private Encoder m_Encoder;

    public Elevator() {
        m_Motor = new CANSparkMax(ElevatorConstants.Motor, MotorType.kBrushless);
        // m_Encoder = new Encoder(2);
    }

    public void setVoltage(double voltage) {
        m_Motor.setVoltage(voltage);
    }

  


    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Elevator Encoder", m_Encoder.get());
    }
    

   
}
