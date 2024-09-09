package frc.robot.Commands.Amp;

import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.subsystems.AmpRollers;

public class AmpScore extends Command {

    private final AmpRollers m_AmpRollers;

    /**
    * Run amp rollers until game piece leaves
    */
    public AmpScore(AmpRollers rollers) {
        m_AmpRollers = rollers;
    }

    @Override
    public void initialize() {
        m_AmpRollers.setVoltage(AmpRollerConstants.IntakeVoltage);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_AmpRollers.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return !m_AmpRollers.gamePieceStored();
    }
    
}
