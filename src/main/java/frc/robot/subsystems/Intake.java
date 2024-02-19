package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

    private CANSparkMax MotorA;


    public Intake() {
        MotorA = new CANSparkMax(IntakeConstants.IntakeMotor, MotorType.kBrushless);

    } 

    public void runIntake(double spd) {
        MotorA.set(spd);
    }

    public void stopMotor() { 
        MotorA.set(0);
    }

    // TODO: break beam
}