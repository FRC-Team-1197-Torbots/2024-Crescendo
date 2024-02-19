package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmStates;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
    private CANSparkMax ArmMotor;
    private Encoder ArmEncoder;
    private ArmConstants.ArmStates m_ArmStates;

    private double targetPos;

    private double armSpeed;
    private TrapezoidProfile.Constraints m_Constraints;
    private ProfiledPIDController m_armPIDController;

    public Arm(){
        ArmMotor = new CANSparkMax(ArmConstants.MotorTop, MotorType.kBrushless);
        ArmEncoder = new Encoder(2, 3);
        m_ArmStates = ArmStates.STORE;
        targetPos = ArmConstants.StorePos;
        m_Constraints = new TrapezoidProfile.Constraints(ArmConstants.MaxAngularVelo, ArmConstants.MaxAngularAccel);
        m_armPIDController = new ProfiledPIDController(ArmConstants.Arm_kP, ArmConstants.Arm_kI, ArmConstants.Arm_kD, m_Constraints);
    }
    
    @Override
    public void periodic() {    
        switch(m_ArmStates){
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
        
    }
    
    public void runArm(double spd ){
        ArmMotor.set(spd);
    }

    public void stopMotor(){ 
        ArmMotor.set(0);
    }

    public double setArmSpeed(){
        return m_armPIDController.calculate(ArmEncoder.getDistance(), targetPos);
    }

    public void getPose() {
        LimelightHelpers.getBotPose(getName());
    }
}