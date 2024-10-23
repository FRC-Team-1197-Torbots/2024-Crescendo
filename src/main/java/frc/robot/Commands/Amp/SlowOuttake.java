package frc.robot.Commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AmpRollers;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
public class SlowOuttake extends Command{
    private Shooter m_Shooter;
    private AmpRollers m_AmpRollers;
    private double voltage;
    public SlowOuttake(Shooter shooter) {
        m_Shooter = shooter;
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {    
        m_Shooter.setTargetRPM(200);
        m_Shooter.runIntake(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.runIntake(0);
        m_Shooter.setTargetRPM(ShooterConstants.IdleSpeed);
    }

    @Override
    public boolean isFinished() {
        return !m_Shooter.gamePieceStored();
    }
}
