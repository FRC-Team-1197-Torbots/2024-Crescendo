package frc.robot.Commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
public class AutoAlign extends Command {

  
    private DriveSubsystem m_RobotDrive;
    private double targetAngle;

    public AutoAlign(DriveSubsystem drive) {
        m_RobotDrive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        targetAngle =  m_RobotDrive.gyroAngle() - LimelightHelpers.getTY("limelight-left");
    }

    @Override
    public void execute() {
        m_RobotDrive.pointAt(targetAngle);
        // m_RobotDrive.drive(0, 0, m_RobotDrive.getNoteAngleOutput(), false, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_RobotDrive.drive(0,0,0, true, true);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTY("limelight-left")) < 4 && LimelightHelpers.getTY("limelight-left") != 0;
    }
    
}
