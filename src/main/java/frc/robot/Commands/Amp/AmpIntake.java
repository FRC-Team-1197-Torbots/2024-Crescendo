package frc.robot.Commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpRollers;

public class AmpIntake extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final AmpRollers m_AmpRollers;
    private double voltage;

    /**
     * Runs amp rollers at specified voltage
     * 
     * @param voltage to run rollers
     * 
     * Stops rollers when interupted 
     */
    public AmpIntake(AmpRollers rollers, double voltage) {
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
        return m_AmpRollers.gamePieceStored();
    }
}
