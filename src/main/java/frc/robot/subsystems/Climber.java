package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase{
    private CANSparkFlex leftClimberFlex;
    private CANSparkFlex rightClimberFlex;

    public Climber(){
        leftClimberFlex = new CANSparkFlex(ClimberConstants.LeftClimberMotor, MotorType.kBrushless);
        rightClimberFlex = new CANSparkFlex(ClimberConstants.RightClimberMotor, MotorType.kBrushless);
    }

    public void runMotors(double speed){
        leftClimberFlex.set(speed);
        rightClimberFlex.set(speed);
    }

    public void stopMotors() {
        leftClimberFlex.set(0);
        rightClimberFlex.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Climber Current", leftClimberFlex.getOutputCurrent());
        SmartDashboard.putNumber("Right Climber Current", rightClimberFlex.getOutputCurrent());

        SmartDashboard.putNumber("Left Voltage", leftClimberFlex.getBusVoltage());
        SmartDashboard.putNumber("Right Voltage", rightClimberFlex.getBusVoltage());


    }

}