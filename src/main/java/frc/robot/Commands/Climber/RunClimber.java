package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

public class RunClimber extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_Climber;
    public RunClimber(Climber subsystem) {
        m_Climber = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {
        System.out.println("James' Arm Psuedocode Initialized");
        
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        //System.out.println("Going up");
        m_Climber.runMotors();
      }

      @Override
        public void end(boolean interrupted) {
            m_Climber.stopMotors();
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
