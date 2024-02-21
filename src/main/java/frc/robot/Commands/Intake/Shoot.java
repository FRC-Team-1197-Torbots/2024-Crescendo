package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Shoot extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;

    public Shoot(Intake subsystem) {
        m_intake = subsystem;
        //m_Speed = speed;
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
        m_intake.runIntake(0.5);
      }

      @Override
        public void end(boolean interrupted) {
            m_intake.stopMotor();
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_intake.gamePieceStored();
  }
    
}
