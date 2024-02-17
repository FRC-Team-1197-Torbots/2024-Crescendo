package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase{
    private CANSparkFlex leftClimberFlex;
    private CANSparkFlex rightClimberFlex;

    public Climber(){
        leftClimberFlex = new CANSparkFlex(ClimberConstants.LeftClimberMotor, MotorType.kBrushless);
        rightClimberFlex = new CANSparkFlex(ClimberConstants.RightClimberMotor, MotorType.kBrushless);
    }

    public void runMotors(){
        leftClimberFlex.set(0.2);
        rightClimberFlex.set(0.2);
    }

    public void stopMotors() {
        leftClimberFlex.set(0);
        rightClimberFlex.set(0);
    }

    @Override
    public void periodic(){

    }

}