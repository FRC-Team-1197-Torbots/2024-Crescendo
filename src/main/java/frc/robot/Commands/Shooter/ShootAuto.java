package frc.robot.Commands.Shooter;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Intake.Shoot;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class ShootAuto extends Command {

    private Arm m_arm;
    private Shooter m_shooter;
    private boolean hasShot;
    private boolean armReachedTarget = false;

    public ShootAuto(Arm arm, Shooter shooter) {
        m_arm = arm;
        m_shooter = shooter;      
        
        m_shooter.AutoShots++;
    }

    @Override
    public void initialize() {
        
        // TODO Auto-generated method stub
        super.initialize();

        switch(m_shooter.AutoShots){
            case 1:
                m_arm.setStates(ArmStates.AUTOTARGET_1);
                //ShootCount++;
                break;
            case 2:
                m_arm.setStates(ArmStates.AUTOTARGET_3);
                //ShootCount++;
                break;
            case 3:
                m_arm.setStates(ArmStates.AUTOTARGET_2);
                //ShootCount++;
                break;
            case 4:
                m_arm.setStates(ArmStates.AUTOTARGET_3);
                //ShootCount++;
                break;
            }
        //m_arm.setStates(ArmStates.AUTOSPEAKER);

    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();

        m_shooter.runShooter(0.9f);

        if(m_shooter.onTarget() && m_arm.onTarget()) {
            m_shooter.runIntakeShooter();
            hasShot = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);

        m_shooter.stopIntakeShooter();
        m_shooter.stopMotor();
    }


    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return !m_shooter.getBreakBeamState();
    }
    
}
