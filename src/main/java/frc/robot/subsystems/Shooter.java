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
    private double top = 0;
    private double bot = 4;
    private double low = 2100;
    private double high = 2150;
    public Shooter(Intake intake) {
        TopMotor = new CANSparkFlex(ShooterConstants.TopMotor, MotorType.kBrushless);
        BottomMotor = new CANSparkFlex(ShooterConstants.BottomMotor, MotorType.kBrushless);
        m_Intake = intake;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Top Flywheel Velocity", TopMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Velocity", BottomMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("low", low);
        SmartDashboard.putNumber("high", high);
        SmartDashboard.putNumber("bot", bot);
        SmartDashboard.putBoolean("Amp On Target", ampOnTarget());
        // SmartDashboard.putNumber(" ,Shooter Kp", shooterKp);
    } 

    public void runShooter(double spd){
        TopMotor.set(-spd);
        BottomMotor.set(-spd); 
    }
    public void runShooter(double top, double bottom){
        TopMotor.setVoltage(-top);
        BottomMotor.setVoltage(-bottom); 
    }

    public void stopMotor() { 
        TopMotor.set(0);
        BottomMotor.set(0);
    }
    public void incrementbot(double amount){
        bot += amount;
    }
    public void incrementrpm(double amount){
        low += amount;
        high += amount;
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
        return BottomMotor.getEncoder().getVelocity();
    }

    public boolean getBreakBeamState(){
        return m_Intake.gamePieceStored();
    }

    public void runIntakeShooter(){
        m_Intake.runIntake(0.5);
    }

    public void stopIntakeShooter(){
        m_Intake.stopMotor();
    }
    

    public boolean onTarget(){
        return Math.abs(getShooterRPM()) > 4500f;
    }
    public boolean ampOnTarget(){
        return Math.abs(getShooterRPM()) > low && Math.abs(getShooterRPM()) < high && Math.abs(TopMotor.getEncoder().getVelocity())< 50;
    }

    public void setMotorMode(IdleMode mode){
        TopMotor.setIdleMode(mode);
        BottomMotor.setIdleMode(mode);
    }



    
}