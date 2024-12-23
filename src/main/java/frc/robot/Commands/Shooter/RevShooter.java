package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RevShooter extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_Shooter;
    private int targetRPM;

    public RevShooter(Shooter subsystem, int rpm) {
        m_Shooter = subsystem;
        targetRPM = rpm;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      m_Shooter.resetTimer();
      m_Shooter.setTargetRPM(targetRPM);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
      m_Shooter.setTargetRPM(0);
      m_Shooter.stopTimer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }  
}
