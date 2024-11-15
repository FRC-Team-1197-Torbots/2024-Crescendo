package frc.robot.Commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.LimelightHelpers;

public class FindNote extends Command {

    private DriveSubsystem m_RobotDrive;
    private double speed;

    public FindNote(DriveSubsystem drive, double speed) {
        m_RobotDrive = drive;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_RobotDrive.drive(0,0,0.1,true, true);
    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return LimelightHelpers.getTY("limelight-left") != 0;
    }
    
}
