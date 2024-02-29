package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class ShootAuto extends Command {

    private Arm m_arm;
    private Shooter m_shooter;
    

    public ShootAuto(Arm arm, Shooter shooter) {
        m_arm = arm;
        m_shooter = shooter;        
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);

        m_shooter.stopIntakeShooter();
        m_shooter.stopMotor();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();

        m_shooter.runShooter(0.85f);

        if(m_shooter.onTarget()) {
            m_shooter.runIntakeShooter();
        }
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();

        m_arm.setStates(ArmStates.AUTOSPEAKER);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return !m_shooter.getBreakBeamState() && m_arm.onTarget();
    }
    
}
