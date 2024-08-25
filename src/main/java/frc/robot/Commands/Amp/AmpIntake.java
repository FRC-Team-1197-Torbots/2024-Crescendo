package frc.robot.Commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.subsystems.AmpRollers;

public class AmpIntake extends Command {

    private final AmpRollers m_AmpRollers;

    public AmpIntake(AmpRollers rollers) {
        m_AmpRollers = rollers;
    }

    @Override
    public void initialize() {
        m_AmpRollers.setTargetRPM(AmpRollerConstants.PassSpeed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
