package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private Intake m_Intake;
    private Timer timer;
    private CANSparkFlex TopMotor;
    private CANSparkFlex BottomMotor;
    private boolean atTargetRPM;
    private double targetRPM;
    public int AutoShots;
    private PIDController m_TopPidController;
    private PIDController m_BottomPidController;

    public Shooter(Intake intake) {
        TopMotor = new CANSparkFlex(ShooterConstants.TopMotor, MotorType.kBrushless);
        BottomMotor = new CANSparkFlex(ShooterConstants.BottomMotor, MotorType.kBrushless);
        m_Intake = intake;
        timer = new Timer();
        targetRPM = 0;        
        AutoShots = 0;
        m_TopPidController = new PIDController(ShooterConstants.kP ,ShooterConstants.kI, ShooterConstants.kD);
        m_BottomPidController = new PIDController(ShooterConstants.kP ,ShooterConstants.kI, ShooterConstants.kD);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter On Target", onTarget());
        SmartDashboard.putNumber("Shooter RPM", getAverageShooterRPM());
        SmartDashboard.putNumber("PID Output", getBottomPIDOutput());
        double feedForward = (double)targetRPM / ShooterConstants.VortexMaxSpeed * ShooterConstants.NominalBatteryVoltage; // funny alex math
        TopMotor.setVoltage(feedForward + getTopPIDOutput());        
        BottomMotor.setVoltage(feedForward + getBottomPIDOutput());        
    }

    @Deprecated
    public void runShooter(double voltage) {
        TopMotor.setVoltage(voltage);
        BottomMotor.setVoltage(voltage);
    }

    public double getBottomPIDOutput(){
        return m_BottomPidController.calculate(getBottomShooterRPM() - targetRPM); // this is the wrong way but it works WPI SUX
    }
    public double getTopPIDOutput(){
        return m_TopPidController.calculate(getTopShooterRPM() - targetRPM); // this is the wrong way but it works WPI SUX
    }

    public void resetTimer() {
        timer.start();
        timer.reset();
    }

    public void stopTimer() {
        timer.reset();
        timer.stop();
    }

    public void runShooter(double top, double bottom) {
        TopMotor.setVoltage(top);
        BottomMotor.setVoltage(bottom);
    }

    public void setTargetRPM(int RPM) {
        targetRPM = RPM;
    }

    public void resetAutoShots() {
        AutoShots = 0;
    }

    public void incrementShotCount() {
        AutoShots++;
    }

    public void stopMotor() {
        setTargetRPM(0);
    }

    public void idleMotor() {
        setTargetRPM(ShooterConstants.IdleSpeed);
    }

    private double getBottomShooterRPM() {
        return BottomMotor.getEncoder().getVelocity();
    }

    private double getTopShooterRPM() {
        return TopMotor.getEncoder().getVelocity();
    }

    public double getAverageShooterRPM() {
        return (getBottomShooterRPM() + getTopShooterRPM()) / 2;
    }

    public boolean gamePieceStored() {
        return m_Intake.gamePieceStored();
    }

    public void runIntakeShooter() {
        m_Intake.runIntake(0.5);
    }
    
    public void runIntake(double speed) {
        m_Intake.runIntake(speed);
    }

    public void stopIntakeShooter() {
        m_Intake.stopMotor();
    }

    public boolean onTarget() {
        atTargetRPM = Math.abs(getAverageShooterRPM() - targetRPM) < 200;
        return atTargetRPM || timer.hasElapsed(3); 
    }

    public void setMotorMode(IdleMode mode) {
        TopMotor.setIdleMode(mode);
        BottomMotor.setIdleMode(mode);
    }
}