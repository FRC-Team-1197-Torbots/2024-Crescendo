package frc.robot.Commands.Shooter;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Intake.Shoot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class ShootAuto extends Command {

    private Arm m_arm;
    private Shooter m_Shooter;
    private boolean armReachedTarget = false;
    private double[] autoTargets;
    private int shots;

    public ShootAuto(Arm arm, Shooter shooter) {
        m_arm = arm;
        m_Shooter = shooter;      
        shots = m_Shooter.AutoShots;    
        autoTargets = m_arm.autoTargets;
    }

    @Override
    public void initialize() {
        
        super.initialize();

        if (shots < autoTargets.length)
            m_arm.setTargetAngle(autoTargets[shots]);
    }

    @Override
    public void execute() {
        super.execute();

        m_Shooter.runShooter(0.9f);

        if(m_Shooter.onTarget() && m_arm.autoOnTarget()) {
            m_Shooter.runIntakeShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        m_Shooter.incrementShotCount();
        m_Shooter.stopIntakeShooter();
        m_Shooter.stopMotor();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return !m_Shooter.getBreakBeamState();
        // return true;
    }
    
}
