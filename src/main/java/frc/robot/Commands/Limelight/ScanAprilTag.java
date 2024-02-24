package frc.robot.Commands.Limelight;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;


public class ScanAprilTag extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_robotDrive;

  public ScanAprilTag(DriveSubsystem subsystem) {
      m_robotDrive = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
      
    }

    @Override
    public void initialize() {
      System.out.println("ScanAprilTag run!");
      
      double[] botpose_intake = LimelightHelpers.getBotPose_wpiBlue("limelight-intake");
      double[] botpose_shooter = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");

      m_robotDrive.resetOdometry(new Pose2d(botpose_shooter[0], botpose_shooter[1], new Rotation2d(Math.toRadians(botpose_shooter[5]))));
      
      //eventually, make this command triggered when picking up note AND when AprilTag(s) is visible 
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
      public void end(boolean interrupted) {
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }  
}
