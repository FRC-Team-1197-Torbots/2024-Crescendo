package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

    private CANSparkFlex MotorA;
    private CANSparkFlex MotorB;
    private DigitalInput breakBeam;

    private boolean gamePieceStored;

    public Shooter() {
        MotorA = new CANSparkFlex(ShooterConstants.TopMotor, MotorType.kBrushless);
        MotorB = new CANSparkFlex(ShooterConstants.BottomMotor, MotorType.kBrushless);
    }

    @Override
    public void periodic(){
        
    } 

    public void runShooter(double spd){
        MotorA.set(spd);
        MotorB.set(-spd); 
    }

    public void stopMotor() { 
        MotorA.set(0);
        MotorB.set(0);
    }

    public boolean breakBeamState(){
        return breakBeam.get();
    }

    
}