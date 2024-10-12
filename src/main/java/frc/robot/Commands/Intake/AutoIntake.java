package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {

    private Arm m_Arm;
    private Intake m_Intake;

    public AutoIntake(Arm arm, Intake intake) {
        m_Arm = arm;
        m_Intake = intake;
    }

    @Override
    public void initialize() {        
        m_Arm.setTargetAngle(ArmConstants.IntakePos);;
        m_Intake.runIntake(IntakeConstants.IntakeSpeed);
    }
    
    @Override
    public void execute() {
    }
    
    
    @Override
    public void end(boolean interrupted) {
        m_Intake.stopMotor();
        m_Arm.setTargetAngle(ArmConstants.StorePos);
    }

    @Override
    public boolean isFinished() {
        return m_Intake.gamePieceStored();
    }
    
}
