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
    private double TopFlyWheelTestVoltage = 0;
    public double BotFlyWheelTestVoltage = 4.2;
    private double low = 2100;
    private double high = 2500;

    public int AutoShots;

    public Shooter(Intake intake) {
        TopMotor = new CANSparkFlex(ShooterConstants.TopMotor, MotorType.kBrushless);
        BottomMotor = new CANSparkFlex(ShooterConstants.BottomMotor, MotorType.kBrushless);
        m_Intake = intake;

        AutoShots = 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Flywheel Velocity", TopMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Velocity", BottomMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("low", low);
        // SmartDashboard.putNumber("high", high);
        // SmartDashboard.putNumber("bot", bot);
        // SmartDashboard.putBoolean("Amp On Target", ampOnTarget());
        SmartDashboard.putBoolean("Shooter RPM on Target", onTarget());
        SmartDashboard.putNumber("Bottom flywheel voltage", BotFlyWheelTestVoltage);
        // SmartDashboard.putNumber(" ,Shooter Kp", shooterKp);
    }

    public void runShooter(double spd) {
        // TopMotor.setVoltage(spd);
        // BottomMotor.setVoltage(spd);
        TopMotor.set(-spd);
        BottomMotor.set(-spd);
    }

    public void runShooter(double top, double bottom) {
        TopMotor.setVoltage(-top);
        BottomMotor.setVoltage(-bottom);
    }

    public void incrementShotCount(){
        AutoShots++;
    }

    public void stopMotor() {
        TopMotor.set(0);
        BottomMotor.set(0);
    }

    public void idleMotor() {
        TopMotor.set(ShooterConstants.IdleSpeed);
        BottomMotor.set(ShooterConstants.IdleSpeed);
    }

    private double getBottomShooterRPM() {
        return BottomMotor.getEncoder().getVelocity();
    }

    private double getTopShooterRPM() {
        return TopMotor.getEncoder().getVelocity();
    }

    public double getAverageShooterRPM(){
        return (getBottomShooterRPM() + getTopShooterRPM()) / 2;
    }

    public boolean getBreakBeamState() {
        return m_Intake.gamePieceStored();
    }

    public void runIntakeShooter() {
        m_Intake.runIntake(0.5);
    }

    public void stopIntakeShooter() {
        m_Intake.stopMotor();
    }

    public boolean onTarget() {
        return Math.abs(getBottomShooterRPM()) > 4500 && Math.abs(getTopShooterRPM()) > 4500;// 4750
    }

    public boolean ampOnTarget() {
        return Math.abs(getBottomShooterRPM()) > low && Math.abs(getBottomShooterRPM()) < high
                && Math.abs(TopMotor.getEncoder().getVelocity()) < 50;
    }

    public void setMotorMode(IdleMode mode) {
        TopMotor.setIdleMode(mode);
        BottomMotor.setIdleMode(mode);
    }

    // TEST CODE
    public void incrementbot(double amount) {
        BotFlyWheelTestVoltage += amount;
    }

    public void incrementrpm(double amount) {
        low += amount;
        high += amount;
    }

    public double getKp() {
        return shooterKp;
    }

    public void incrementKp(double amount) {
        shooterKp += amount;
    }

}