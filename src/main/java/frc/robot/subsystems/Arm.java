package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.LimelightHelpers;

public class Arm extends SubsystemBase {
    private CANSparkFlex ArmMotor1;
    private CANSparkFlex ArmMotor2;
    private Encoder ArmEncoder;

    private double targetPos = ArmConstants.StorePos;

    private DriveSubsystem m_DriveSubsystem;
    private PIDController m_PIDController;
    private double error;
    public double[] autoTargets;
    public int ShuttleRPM = ShooterConstants.ShuttleRPM;
    private boolean pidActive = true;

    public Arm(DriveSubsystem drive) {
        SmartDashboard.putNumber("Target Angle", targetPos);
        SmartDashboard.putNumber("Shuttle RPM", ShooterConstants.ShuttleRPM);

        ArmMotor1 = new CANSparkFlex(ArmConstants.Motor1, MotorType.kBrushless);
        ArmMotor2 = new CANSparkFlex(ArmConstants.Motor2, MotorType.kBrushless);

        ArmMotor1.setSmartCurrentLimit(30);
        ArmMotor2.setSmartCurrentLimit(30);
        ArmEncoder = new Encoder(ArmConstants.encoderChannelA, ArmConstants.encoderChannelB, true, EncodingType.k4X);
        ArmEncoder.reset();
        m_PIDController = new PIDController(ArmConstants.Arm_kP, ArmConstants.Arm_kI, ArmConstants.Arm_kD);
        
        // ArmConstants.Arm_kI, ArmConstants.Arm_kD, m_Constraints);
        // m_ArmFeedforward = new ArmFeedforward(0.1, 0.66, 1.30, 0.02); // ks value might need to change
        /****************************
         * 
         * 
         * Feedforward numbers based off of this link:
         * https://www.reca.lc/arm?armMass=%7B%22s%22%3A29.608%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A9.0826%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A83%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%20Vortex%2A%22%7D&ratio=%7B%22magnitude%22%3A76.8%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
         * 
         * 
         **************************/

        m_DriveSubsystem = drive;
    }

    @Override
    public void periodic() {
        if(pidActive)
            runPID();
        SmartDashboard.putNumber("Arm Angle", getRadians());
    }

    public void runPID() {
        error = targetPos - getRadians();
        double armVoltage = setArmOutput();
        runArm(armVoltage);
    }

    public void setTargetAngle(double target) {
        targetPos = MathUtil.clamp(target, -0.2, 1.83);
    }

    public void setZeroingArm(boolean pidActive) {
        this.pidActive = pidActive;
    }

    public void updateFromSmartDashboard() {
        ShooterConstants.ShuttleRPM = (int)SmartDashboard.getNumber("Shuttle RPM", ShooterConstants.ShuttleRPM);
    }

    public double setAngleFromDistance() {
        double distance = distanceFromSpeaker();
        double result = 0.155 + 0.336*distance + -0.0257*distance*distance + -0.00313*distance*distance*distance;
        // double result = 0.169 + 0.336*distance + -0.0257*distance*distance + -0.00313*distance*distance*distance;
        SmartDashboard.putNumber("Calculated Angle", result);
        return MathUtil.clamp(result, ArmConstants.SubwooferPos, ArmConstants.StorePos);

    }

    public void runArm(double voltage) {
        ArmMotor1.setVoltage(voltage);
        ArmMotor2.setVoltage(voltage);
    }

    public void stopMotor() {
        ArmMotor1.set(0);
        ArmMotor2.set(0);
    }

    public void idleSpeed() {
        ArmMotor1.set(-0.01);
        ArmMotor2.set(-0.01);
    }

    public boolean onTarget() {
        return error > -3 && error < 1.5; // if error is between -5 and 1
    }
    
    public boolean autoOnTarget() {
        return error > -4 && error < 1; // if error is between -5 and 1
    }

    public void setAutoTargets(String autoName) {
        SmartDashboard.putString("Auto Selected", autoName);
        switch (autoName) {
            case("2 Note Middle"):
                autoTargets = ArmConstants.TwoNoteMiddleTargets;
                break;
            case("1 Note Top"):
                autoTargets = ArmConstants.OneNoteTopTargets;
                break;
            case("1 Note Bottom"):
                autoTargets = ArmConstants.OneNoteBottomTargets;
                break;
            case("2 Note Bottom Red"):
                autoTargets = ArmConstants.TwoNoteBottomTargets;
                break;
            case("2 Note Bottom Blue"):
                autoTargets = ArmConstants.TwoNoteBottomTargets;
                break;
            case("2 Note Bottom Shrunk"):
                autoTargets = ArmConstants.TwoNoteBottomTargets;
                break;
            case("2 Note Bottom to Inner Center"):
                autoTargets = ArmConstants.TwoNoteBottomInnerCenterTargets;
                break;
            case("3 Note Bottom to Inner Center"):
                autoTargets = ArmConstants.ThreeNoteBottomInnerCenterTargets;
                break;
            case("3 Note Bottom Red"):
                autoTargets = ArmConstants.ThreeNoteBottomTargets;
                break;
            case("3 Note Bottom Blue"):
                autoTargets = ArmConstants.ThreeNoteBottomTargets;
                break;
            case("3 Note Middle"):
                autoTargets = ArmConstants.ThreeNoteMiddleTargets;
                break;
            case("3 Note Top to Center"):
                autoTargets = ArmConstants.ThreeNoteTopCenterTargets;
                break;
            case("4 Note Top"):
                autoTargets = ArmConstants.FourNoteTopTargets;
                break;
            case("4 Note Bottom"):
                autoTargets = ArmConstants.FourNoteBottomTargets;
                break;
            case("4 Note Middle Red"):
                autoTargets = ArmConstants.FourNoteMiddleTargetsRed;
                break;
            case("4 Note Middle Blue"):
                autoTargets = ArmConstants.FourNoteMiddleTargetsBlue;
                break;
            case("4 Note Middle Reverse Red"):
                autoTargets = ArmConstants.FourNoteMiddleReverseTargetsRed;
                break;
            case("4 Note Middle Reverse Blue"):
                autoTargets = ArmConstants.FourNoteMiddleReverseTargetsBlue;
                break;
            case("5 Note Top"):
                autoTargets = ArmConstants.FourNoteTopTargets;
                break;
            case("5 Note Middle"):
                autoTargets = ArmConstants.FiveNoteMiddleTargets;
                break;
            default:
                autoTargets = ArmConstants.StoreTargets;
                break;
        } 
        // SmartDashboard.putNumberArray("Auto targets", autoTargets);
    }

    // public void setAngle(double angle) {
    //     testAngle = angle;
    // }

    public void resetArm() {
        ArmEncoder.reset();
    }

    public void toggleIntake() {
        if (targetPos == ArmConstants.IntakePos)
            targetPos = ArmConstants.StorePos;
        else
            targetPos = ArmConstants.IntakePos;
    }

    public double setArmOutput() {
        return -ArmConstants.FeedForwardGravity * Math.cos(getRadians()) + m_PIDController.calculate(error); 
    }

    public double getRadians() {
        double rawValue = (double)ArmEncoder.get() / ArmConstants.TicksPerRevolution * 2.0 * Math.PI / ArmConstants.EncoderToArmGear;
        return rawValue + ArmConstants.HardStopOffset;
    }

    public void getPose() {
        LimelightHelpers.getBotPose(getName());
    }

    public double distanceFromSpeaker() {
        return m_DriveSubsystem.distanceFromSpeaker();
    }

    public void setMotorMode(IdleMode mode) {
        ArmMotor1.setIdleMode(mode);
        ArmMotor2.setIdleMode(mode);
    }

    public double getArmOutput() {
        return (ArmMotor1.getOutputCurrent() + ArmMotor2.getAppliedOutput()) / 2;
    }

    public void updateAmpTarget() {
        ArmConstants.AmpPos = SmartDashboard.getNumber("Arm Amp Angle", ArmConstants.AmpPos);
    }

    public boolean onAmpTarget() {
        return Math.abs(getRadians() - ArmConstants.AmpPos) < 0.07;
    }
}