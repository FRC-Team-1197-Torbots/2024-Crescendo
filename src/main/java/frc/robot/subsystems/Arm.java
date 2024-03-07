package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private CANSparkFlex ArmMotor1;
    private CANSparkFlex ArmMotor2;
    private Encoder ArmEncoder;

    private double targetPos = ArmConstants.StorePos;
    private double armKp;
    private double armKi;
    private double armKd;
    private double armVoltage;
    // private TrapezoidProfile.Constraints m_Constraints;
    // private ProfiledPIDController m_armPIDController;
    private PIDController m_PIDController;
    // private ArmFeedforward m_ArmFeedforward;
    private double error;
    public double testAngle;
    // private double feedForward;
    public double[] autoTargets;

    private DriveSubsystem m_DriveSubsystem;
    private Limelight m_Limelight;

    public Arm(DriveSubsystem drive, Limelight limelight) {
        testAngle = ArmConstants.TestPos;
        ArmMotor1 = new CANSparkFlex(ArmConstants.Motor1, MotorType.kBrushless);
        ArmMotor2 = new CANSparkFlex(ArmConstants.Motor2, MotorType.kBrushless);

        ArmMotor1.setSmartCurrentLimit(30);
        ArmMotor2.setSmartCurrentLimit(30);

        ArmEncoder = new Encoder(ArmConstants.encoderChannelA, ArmConstants.encoderChannelB, false, EncodingType.k4X);
        ArmEncoder.reset();
        // m_Constraints = new TrapezoidProfile.Constraints(ArmConstants.MaxAngularVelo, ArmConstants.MaxAngularAccel);
        m_PIDController = new PIDController(ArmConstants.Arm_kP, ArmConstants.Arm_kI, ArmConstants.Arm_kD);
        armKp = ArmConstants.Arm_kP;
        armKi = ArmConstants.Arm_kI;
        armKd = ArmConstants.Arm_kD;
        // feedForward = 0.001;
        // m_armPIDController = new ProfiledPIDController(ArmConstants.Arm_kP,
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
        m_Limelight = limelight;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", ticksToDegrees(ArmEncoder.get()));
        SmartDashboard.putNumber("Target Angle", targetPos);
        // SmartDashboard.putBoolean("Arm On Target", onTarget());
        SmartDashboard.putNumber("Test Angle", testAngle);
        // SmartDashboard.putNumber("Arm speed", armSpeed);
        // SmartDashboard.putNumber("Arm Kp", armKp);

        // SmartDashboard.putNumber("Arm Ki", armKi);
        // SmartDashboard.putNumber("Arm Kd", armKd);
        // SmartDashboard.putNumber("Get Arm Angular Velo", getAngularVelo());
        // SmartDashboard.putNumber("Error", error);
    
        error = targetPos - ticksToDegrees(ArmEncoder.get());
        armVoltage = setArmOutput();
        runArm(armVoltage);
    }

    public void setTargetAngle(double target) {
        targetPos = target;
    }
    
    public double setAngleFromDistance() {
        double distance = distanceFromSpeaker();
        double testAngle = ArmConstants.A * Math.log(distance) + ArmConstants.B;
        if (testAngle < ArmConstants.StorePos) {
            testAngle = ArmConstants.StorePos;
        }
        if (testAngle > ArmConstants.IntakePos) {
            testAngle = ArmConstants.IntakePos;
        }
        return testAngle;
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
                autoTargets = ArmConstants.TwoNoteMidTargets;
                break;
            case("1 Note Top"):
                autoTargets = ArmConstants.OneNoteTopTargets;
                break;
            case("1 Note Bottom"):
                autoTargets = ArmConstants.OneNoteBottomTargets;
                break;
            case("3 Note Middle"):
                autoTargets = ArmConstants.ThreeNoteMidTargets;
                break;
            case("4 Note Top"):
                autoTargets = ArmConstants.FourNoteTopTargets;
                break;
            case("5 Note Top"):
                autoTargets = ArmConstants.FourNoteTopTargets;
                break;
            default:
                autoTargets = ArmConstants.StoreTargets;
                break;
        } 
        SmartDashboard.putNumberArray("Auto targets", autoTargets);
    }

    // public void setAngle(double angle) {
    //     testAngle = angle;
    // }

    public void resetArm() {
        ArmEncoder.reset();
    }

    public void toggleIntake() {
        if(targetPos == ArmConstants.IntakePos)
            targetPos = ArmConstants.StorePos;
        else
            targetPos = ArmConstants.IntakePos;
    }

    public double getAngularVelo() {
        return ArmEncoder.getRate() / ArmConstants.TICKS_PER_DEGREE;
    }

    public double setArmOutput() {
        return m_PIDController.calculate(ticksToDegrees(ArmEncoder.get()) - targetPos);
    }

    public double ticksToDegrees(double ticks) {
        return ticks / ArmConstants.TICKS_PER_DEGREE;
    }

    public void getPose() {
        LimelightHelpers.getBotPose(getName());
    }

    public double distanceFromSpeaker() {
        return m_DriveSubsystem.distanceFromSpeaker();
    }

    public double autoDistanceFromSpeaker() {
        return m_Limelight.distanceFromSpeaker();
    }

    public void setMotorMode(IdleMode mode) {
        ArmMotor1.setIdleMode(mode);
        ArmMotor2.setIdleMode(mode);
    }

    // TEST CODE
    public void incrementAngle(double amount) {
        testAngle += amount;
    }

    public double getKp() {
        return armKp;
    }

    public void incrementKp(double amount) {
        armKp += amount;
        m_PIDController.setP(armKp);
    }

    public void incrementKi(double amount) {
        armKi += amount;
        m_PIDController.setI(armKi);
    }

    public void incrementKd(double amount) {
        armKd += amount;
        m_PIDController.setD(armKd);
    }
}