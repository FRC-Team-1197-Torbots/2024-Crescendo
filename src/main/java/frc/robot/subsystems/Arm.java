package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {

    private CANSparkMax MotorA;
    private CANSparkMax MotorB;

    public Arm() {
        MotorA = new CANSparkMax(ArmConstants.MotorA, MotorType.kBrushless);
        MotorB = new CANSparkMax(ArmConstants.MotorB, MotorType.kBrushless);
    } 

    public void runArm(double spd ) {
        MotorA.set(spd);
        MotorB.set(spd);
    }

    public void stopMotor() { 
        MotorA.set(0);
        MotorB.set(0);
    }

    public void getPose() {
        LimelightHelpers.getBotPose(getName());
    }
}