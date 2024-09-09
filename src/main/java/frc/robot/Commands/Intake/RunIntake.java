package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_Intake;
    private final double m_Speed;
    private WaitCommand wait;
    //private double m_Speed;
    public RunIntake(Intake subsystem, double spd) {
        m_Intake = subsystem;
        m_Speed = spd;
        
        //m_Speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {
        m_Intake.setFinishedIntake(false);
        
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        //System.out.println("Going up");
        m_Intake.runIntake(m_Speed);
      }

      @Override
        public void end(boolean interrupted) {
          m_Intake.stopMotor();
          m_Intake.setFinishedIntake(true);
        }
      
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Intake.gamePieceStored();
  }
    
}
