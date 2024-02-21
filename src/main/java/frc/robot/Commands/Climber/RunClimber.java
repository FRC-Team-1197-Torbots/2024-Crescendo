package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

public class RunClimber extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_Climber;
    private double m_Speed;
    public RunClimber(Climber subsystem, double speed) {
        m_Climber = subsystem;
        m_Speed = speed;
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
        m_Climber.runMotors(m_Speed);
      }

      @Override
        public void end(boolean interrupted) {
            m_Climber.stopMotors();
            m_Climber.resetLeft();
            m_Climber.resetRight();
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
