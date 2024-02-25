package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
    private Intake m_Intake;

    private CANSparkFlex TopMotor;
    private CANSparkFlex BottomMotor;
    private double shooterKp = 0.001;

    public Shooter() {
        TopMotor = new CANSparkFlex(ShooterConstants.TopMotor, MotorType.kBrushless);
        BottomMotor = new CANSparkFlex(ShooterConstants.BottomMotor, MotorType.kBrushless);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Top Flywheel Velocity", TopMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Velocity", BottomMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Shooter Kp", shooterKp);
    } 

    public void runShooter(double spd){
        TopMotor.set(-spd);
        BottomMotor.set(-spd); 
    }

    public void stopMotor() { 
        TopMotor.set(0);
        BottomMotor.set(0);
    }

    public void idleMotor() { 
        TopMotor.set(ShooterConstants.IdleSpeed);
        BottomMotor.set(ShooterConstants.IdleSpeed);
    }

    public double getKp() {
        return shooterKp;
    }
    public void incrementKp(double amount) {
        shooterKp += amount;
    }

    public double getShooterRPM(){
        return TopMotor.getEncoder().getVelocity();
    }

    public void getIntakeSubsystem(Intake intake){
        m_Intake = intake;
    }

    public boolean getBreakBeamState(){
        return m_Intake.gamePieceStored();
    }

    public boolean onTarget(){
        return getShooterRPM() < -4500;
    }

    public void setMotorMode(IdleMode mode){
        TopMotor.setIdleMode(mode);
        BottomMotor.setIdleMode(mode);
    }



    
}