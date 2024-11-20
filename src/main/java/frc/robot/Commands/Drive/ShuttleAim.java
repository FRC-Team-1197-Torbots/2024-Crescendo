package frc.robot.Commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ShuttleAim extends Command {

    private DriveSubsystem m_RobotDrive;
    private CommandXboxController m_driverController;

    public ShuttleAim(DriveSubsystem drive, CommandXboxController controller) {
        m_RobotDrive = drive;
        m_driverController = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
       m_RobotDrive.drive(
      -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
      m_RobotDrive.getShuttleRotationSpeed(), 
      true, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
