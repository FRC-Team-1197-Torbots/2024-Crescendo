package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase{
    private CANSparkFlex leftClimberFlex;
    private CANSparkFlex rightClimberFlex;
    private boolean leftAtBottom = false;
    private boolean rightAtBottom = false;

    public Climber(){
        leftClimberFlex = new CANSparkFlex(ClimberConstants.LeftClimberMotor, MotorType.kBrushless);
        rightClimberFlex = new CANSparkFlex(ClimberConstants.RightClimberMotor, MotorType.kBrushless);

        leftClimberFlex.setSmartCurrentLimit(40);
        rightClimberFlex.setSmartCurrentLimit(40);
    }

    public void setMotorMode(IdleMode mode){
        leftClimberFlex.setIdleMode(mode);
        rightClimberFlex.setIdleMode(mode);
    }

    public void runMotors(double speed){
        if(leftClimberFlex.getOutputCurrent() > 40){
            leftAtBottom = true;
        }
        if(rightClimberFlex.getOutputCurrent() > 40){
            rightAtBottom = true;
        }
        if(leftAtBottom)
            leftClimberFlex.set(0);
        else
            leftClimberFlex.set(speed);
        if(rightAtBottom)
            rightClimberFlex.set(0);
        else
            rightClimberFlex.set(speed);
        
    }

    public void stopMotors() {
        leftClimberFlex.set(0);
        rightClimberFlex.set(0);
    }

    public void resetLeft(){
        leftAtBottom = false;
    }

    public void resetRight(){
        rightAtBottom = false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Climber Current", leftClimberFlex.getOutputCurrent());
        SmartDashboard.putNumber("Right Climber Current", rightClimberFlex.getOutputCurrent());

        SmartDashboard.putBoolean("left at Bottom", leftAtBottom);
        SmartDashboard.putBoolean("right at Bottom", rightAtBottom);

        //SmartDashboard.putNumber("Left Voltage", leftClimberFlex.getBusVoltage());
        //SmartDashboard.putNumber("Right Voltage", rightClimberFlex.getBusVoltage());


    }

}