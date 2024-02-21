package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

    private CANSparkMax MotorA;
    private DigitalInput breakBeam;
    private boolean gamePieceStored;

    public Intake() {
        MotorA = new CANSparkMax(IntakeConstants.IntakeMotor, MotorType.kBrushless);
        breakBeam = new DigitalInput(IntakeConstants.breakBeam);

    } 

    public void runIntake(double spd) {
        MotorA.set(-spd);
    }

    public void stopMotor() { 
        MotorA.set(0);
    }
    public boolean gamePieceStored(){
        return !breakBeam.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Break beam", gamePieceStored());
    }
}