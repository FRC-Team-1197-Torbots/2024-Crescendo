package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {

    private Arm m_arm;
    private Intake m_intake;

    public AutoIntake(Arm arm, Intake intake) {
        m_arm = arm;
        m_intake = intake;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);

        m_intake.runIntake(0);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void initialize() {
        super.initialize();

        m_arm.setTargetAngle(ArmConstants.IntakePos);;
        m_intake.runIntake(0.4);
    }

    @Override
    public boolean isFinished() {
        return m_intake.gamePieceStored();
    }
    
}
