package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AimRobot extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private DriveSubsystem m_RobotDrive;
    private double targetAngle;
    //private double m_Speed;
    public AimRobot(DriveSubsystem subsystem) {
        m_RobotDrive = subsystem;
        addRequirements(subsystem);
      }

    @Override
    public void initialize() {
        
        targetAngle = m_RobotDrive.calcAngle();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_RobotDrive.aimRobot(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        m_RobotDrive.drive(0,0,0,false,false);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_RobotDrive.onTarget();
  }
    
}
