package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

    private CANSparkMax MotorA;

    public Climber() {
        MotorA = new CANSparkMax(ClimberConstants.MotorA, MotorType.kBrushless);
    } 

    public void runClimber(double spd){
        MotorA.set(spd);
    }

    public void stopMotor() { 
        MotorA.set(0);
    }
}