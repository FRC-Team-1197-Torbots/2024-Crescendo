package frc.robot.Commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_Shooter;
    private PIDController m_ShooterPID;
    private double targetRPM = -4000;
    public AutoShooter(Shooter subsystem) {
        m_Shooter = subsystem;
        m_ShooterPID = new PIDController(m_Shooter.getKp(), 0.000001, 0);
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
        //System.out.println("revving");
        m_Shooter.runShooter(0.85);
        if(m_Shooter.onTarget()){
          m_Shooter.runIntakeShooter();
        }
        //m_Shooter.runShooter(0.85);
      }

      @Override
        public void end(boolean interrupted) {
          m_Shooter.stopMotor();
          m_Shooter.stopIntakeShooter();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            //System.out.println(!m_Shooter.getBreakBeamState());
            return !m_Shooter.getBreakBeamState();
        }
    
}
