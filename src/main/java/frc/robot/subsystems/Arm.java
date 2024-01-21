package frc.robot.subsystems;
import frc.robot.LimelightHelpers;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
    private CANSparkMax MotorA;
    public Arm(){
        MotorA = new CANSparkMax(ArmConstants.MotorTop, MotorType.kBrushless);
        
    } 
    public void runArm(double spd ){
        MotorA.set(spd);
    }
    public void stopMotor(){ 
        MotorA.set(0);
    }
    public void getPose(){
        LimelightHelpers.getBotPose(getName());
    }
}