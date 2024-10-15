package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class ShootAuto extends Command {

    private Arm m_arm;
    private Shooter m_Shooter;
    private double[] autoTargets;
    private int shots;

    public ShootAuto(Arm arm, Shooter shooter) {
        m_arm = arm;
        m_Shooter = shooter;       
    }

    @Override
    public void initialize() {
        
        super.initialize();
        autoTargets = m_arm.autoTargets;
        shots = m_Shooter.AutoShots;   
        if (shots < autoTargets.length)
            m_arm.setTargetAngle(autoTargets[shots]);
    }

    @Override
    public void execute() {
        super.execute();

        m_Shooter.setTargetRPM(ShooterConstants.ShootingRPM);

        if(m_Shooter.onTarget() && m_arm.autoOnTarget()) {
            m_Shooter.runIntakeShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_Shooter.incrementShotCount();
        m_Shooter.stopIntakeShooter();
        m_Shooter.idleMotor();
    }

    @Override
    public boolean isFinished() {
        return !m_Shooter.gamePieceStored();
    }
    
}
