package frc.robot.Commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.LimelightHelpers;

public class StopDriving extends Command {

    private DriveSubsystem m_RobotDrive;

    public StopDriving(DriveSubsystem drive) {
        m_RobotDrive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_RobotDrive.drive(0,0,0,false, true);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
