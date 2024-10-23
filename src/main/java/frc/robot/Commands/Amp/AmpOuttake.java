package frc.robot.Commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.AmpRollers;

public class AmpOuttake extends Command {
    private Shooter m_Shooter;
    private AmpRollers m_AmpRollers;
    public AmpOuttake(Shooter shooter, AmpRollers ampRollers) {
        m_Shooter = shooter;
        m_AmpRollers = ampRollers;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_Shooter.setTargetRPM(ShooterConstants.OuttakeRPM);        
        m_AmpRollers.setVoltage(4.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_AmpRollers.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return m_Shooter.gamePieceStored();
    }
}
