package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private double targetPos = ElevatorConstants.StorePos;
    private double error, elevatorVoltage;
    private double feedForward = -0.3;
    private CANSparkMax m_Motor;
    private PIDController m_PidController;

    // private Encoder m_Encoder;
    private RelativeEncoder m_Encoder;
    private BooleanSupplier m_BeamBreak;
    private double marginOfError = 0.4;

    public Elevator() {
        
        m_Motor = new CANSparkMax(ElevatorConstants.Motor, MotorType.kBrushless);
        m_Encoder = m_Motor.getEncoder();
        m_PidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        SmartDashboard.putNumber("Elevator target", targetPos);
        SmartDashboard.putNumber("Roller Voltage", AmpRollerConstants.ScoreVoltage);
    }

    public void setVoltage(double voltage) {
        m_Motor.setVoltage(voltage);
    }

    public void setTargetPos(double value) {
        targetPos = value;
    }

    public void giveBeamBreak(BooleanSupplier beamBreak) {
        m_BeamBreak = beamBreak;
    }

    public boolean gamePieceStored() {
        return m_BeamBreak.getAsBoolean();
    }

    public void updateFromSmartDashboard() {
        ElevatorConstants.AmpPos = SmartDashboard.getNumber("Elevator target", ElevatorConstants.AmpPos);
        AmpRollerConstants.ScoreVoltage = SmartDashboard.getNumber("Roller Voltage", AmpRollerConstants.ScoreVoltage);
    }

    public double getPosition() {
        return -m_Encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", getPosition());
        runPID();
    }
    
    public void runPID() {
        error = targetPos - getPosition();
        elevatorVoltage = m_PidController.calculate(error);
        setVoltage(feedForward + elevatorVoltage);
    }

    public boolean atAmpHeight() {
        return Math.abs(error) < marginOfError;
    }
}
