package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

public class AimAtSpeaker extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_RobotDrive;

    public AimAtSpeaker(DriveSubsystem subsystem) {
        m_RobotDrive = subsystem;
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
        m_RobotDrive.aimRobot();
      }

      @Override
        public void end(boolean interrupted) {
            m_RobotDrive.drive(0, 0, 0, true, true);
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_RobotDrive.facingSpeaker() && Math.toRadians(Math.abs(m_RobotDrive.m_gyro.getRate())) < 0.2; //m_RobotDrive.getSpeeds().omegaRadiansPerSecond < 0.2
  }
    
}
