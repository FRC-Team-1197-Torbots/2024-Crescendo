package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class RevShooter extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_Shooter;
    public RevShooter(Shooter subsystem) {
        m_Shooter = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {
        
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        //System.out.println("Going up");
        m_Shooter.runShooter(0.9);
      }

      @Override
        public void end(boolean interrupted) {
          m_Shooter.stopMotor();
            //m_Climber.stopMotors();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }

    
    
}
