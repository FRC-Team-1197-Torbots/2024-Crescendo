package frc.robot.Commands.Amp;

import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.subsystems.AmpRollers;

public class AmpScore extends Command {

    private final AmpRollers m_AmpRollers;
    private double voltage;

    /**
    * Run amp rollers until game piece leaves
    */
    public AmpScore(AmpRollers rollers, double voltage) {
        m_AmpRollers = rollers;
        this.voltage = voltage; 
    }

    @Override
    public void initialize() {
        m_AmpRollers.setVoltage(voltage);
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
