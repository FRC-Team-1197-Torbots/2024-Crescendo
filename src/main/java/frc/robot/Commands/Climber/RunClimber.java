package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimberDirection;
import frc.robot.subsystems.Climber;

public class RunClimber extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_Climber;
  private ClimberDirection m_Direction;
  public RunClimber(Climber subsystem, ClimberDirection direction) {
    m_Climber = subsystem;
    m_Direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climber.runMotors(m_Direction);
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
    return m_Climber.atEndPos(m_Direction);
  }
    
}
