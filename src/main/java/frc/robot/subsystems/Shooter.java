package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

    private CANSparkFlex TopMotor;
    private CANSparkFlex BottomMotor;

    public Shooter() {
        TopMotor = new CANSparkFlex(ShooterConstants.TopMotor, MotorType.kBrushless);
        BottomMotor = new CANSparkFlex(ShooterConstants.BottomMotor, MotorType.kBrushless);
    }

    @Override
    public void periodic(){
        
    } 

    public void runShooter(double spd){
        TopMotor.set(-spd);
        BottomMotor.set(-spd); 
    }

    public void stopMotor() { 
        TopMotor.set(0);
        BottomMotor.set(0);
    }



    
}