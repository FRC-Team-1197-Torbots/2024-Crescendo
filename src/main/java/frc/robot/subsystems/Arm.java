package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmStates;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
    private CANSparkFlex ArmMotor1;
    private CANSparkFlex ArmMotor2;
    private Encoder ArmEncoder;
    private ArmConstants.ArmStates m_ArmStates;

    private double targetPos;

    private double armSpeed;
    private TrapezoidProfile.Constraints m_Constraints;
    private ProfiledPIDController m_armPIDController;

    public Arm() {
        ArmMotor1 = new CANSparkFlex(ArmConstants.Motor1, MotorType.kBrushless);
        ArmMotor2 = new CANSparkFlex(ArmConstants.Motor2, MotorType.kBrushless);
        ArmMotor1.setIdleMode(IdleMode.kBrake);
        ArmMotor2.setIdleMode(IdleMode.kBrake);
        ArmEncoder = new Encoder(ArmConstants.encoderChannelA, ArmConstants.encoderChannelB, false, EncodingType.k4X);
        ArmEncoder.reset();
        m_ArmStates = ArmStates.SPEAKER;
        m_Constraints = new TrapezoidProfile.Constraints(ArmConstants.MaxAngularVelo, ArmConstants.MaxAngularAccel);
        m_armPIDController = new ProfiledPIDController(ArmConstants.Arm_kP, ArmConstants.Arm_kI, ArmConstants.Arm_kD, m_Constraints);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position Encoder", ArmEncoder.get());
        SmartDashboard.putNumber("Arm Angle", ticksToDegrees(ArmEncoder.get()));

        switch(m_ArmStates) {
            case STORE:
                targetPos = ArmConstants.StorePos;
                break;
            case INTAKE:
                targetPos = ArmConstants.IntakePos;
                break;
            case SPEAKER:
                break;
            case AMP:
                break;
            }
        
        armSpeed = setArmSpeed();
        runArm(armSpeed);
        
    }

    public void setToIntake() {
        m_ArmStates = ArmStates.INTAKE;
    }

    public void setToStore() {
        m_ArmStates = ArmStates.STORE;
    }
    
    public void runArm(double spd) {
        ArmMotor1.set(spd);
        ArmMotor2.set(spd);
    }

    public void stopMotor() { 
        ArmMotor1.set(0);
        ArmMotor2.set(0);
    }

    public double setArmSpeed() {
        return m_armPIDController.calculate(ticksToDegrees(ArmEncoder.get()), targetPos);
    }

    public double ticksToDegrees(double ticks) {
        return ticks / ArmConstants.TICKS_PER_DEGREE;
    }

    public void getPose() {
        LimelightHelpers.getBotPose(getName());
    }
}