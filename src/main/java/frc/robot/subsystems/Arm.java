package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
    private CANSparkMax ArmMotor;
    public Arm(){
        ArmMotor = new CANSparkMax(ArmConstants.MotorTop, MotorType.kBrushless);
    } 
    
    public void runArm(double spd ){
        ArmMotor.set(spd);
    }

    public void stopMotor(){ 
        ArmMotor.set(0);
    }

    public void getPose() {
        LimelightHelpers.getBotPose(getName());
    }
}