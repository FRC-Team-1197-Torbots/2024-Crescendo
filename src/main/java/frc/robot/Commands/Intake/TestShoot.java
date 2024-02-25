package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;

public class TestShoot extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    public TestShoot(Intake subsystem) {
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
        m_driverController.leftBumper().whileTrue(new Shoot(m_intake));
        //System.out.println("Going up");
      }

      @Override
        public void end(boolean interrupted) {
            // m_intake.stopMotor();
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
