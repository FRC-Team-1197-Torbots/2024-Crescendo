package frc.robot.Commands.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class AmpShooter extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_Shooter;

    public AmpShooter(Shooter subsystem) {
        m_Shooter = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {
        m_Shooter.runShooter(0, m_Shooter.BotFlyWheelTestVoltage);
        m_Shooter.setMotorMode(IdleMode.kCoast);
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        //System.out.println("Going up");
        // m_Shooter.runShooter(m_ShooterPID.calculate(targetRPM - m_Shooter.getShooterRPM()));
        
      }

      @Override
        public void end(boolean interrupted) {
          m_Shooter.setMotorMode(IdleMode.kBrake);  //m_Climber.stopMotors();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return m_Shooter.ampOnTarget();
        }

    
    
}
