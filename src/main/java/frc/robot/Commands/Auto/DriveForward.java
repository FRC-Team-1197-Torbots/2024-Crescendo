package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.utils.LimelightHelpers;

public class DriveForward extends Command {

    private DriveSubsystem m_RobotDrive;
    private Intake m_Intake;
    private double speed;

    public DriveForward(DriveSubsystem drive, Intake intake, double speed) {
        m_RobotDrive = drive;
        m_Intake = intake; 
        this.speed = speed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTX("limelight-left") > 10)
            m_RobotDrive.drive(-speed,0,0,false, true);
        else {
            m_RobotDrive.drive(speed,0,m_RobotDrive.getNoteAngleOutput(),false, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_RobotDrive.drive(0,0,0,false, true);
    }

    @Override
    public boolean isFinished() {
        return m_Intake.gamePieceStored();
    }
    
}