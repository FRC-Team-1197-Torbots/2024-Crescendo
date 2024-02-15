package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

    private CANSparkMax MotorA;
    private CANSparkMax MotorB;

    public Shooter() {
        MotorA = new CANSparkMax(ShooterConstants.MotorA, MotorType.kBrushless);
        MotorB = new CANSparkMax(ShooterConstants.MotorB, MotorType.kBrushless);
    } 

    public void runShooter(double spd){
        MotorA.set(spd);
        MotorB.set(-spd); 
    }

    public void stopMotor() { 
        MotorA.set(0);
        MotorB.set(0);
    }
}