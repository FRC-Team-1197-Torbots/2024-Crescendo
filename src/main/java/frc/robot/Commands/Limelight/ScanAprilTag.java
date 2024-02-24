package frc.robot.Commands.Limelight;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;


public class ScanAprilTag extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_robotDrive;
  private double[] botpose_intake;
  private double[] botpose_shooter;

  public ScanAprilTag(DriveSubsystem subsystem) {
      m_robotDrive = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      System.out.println("ScanAprilTag run!");
      
      botpose_intake = LimelightHelpers.getBotPose_wpiBlue("limelight-intake");
      botpose_shooter = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");

      m_robotDrive.resetOdometry(new Pose2d(botpose_shooter[0], botpose_shooter[1], new Rotation2d(Math.toRadians(botpose_shooter[5]))));
      //seannys soodocode
      //if we see from limelight, check distance. if distance is too large, dont update. if its good, update

      
      //eventually, make this command triggered when picking up note AND when AprilTag(s) is visible 
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // botpose_intake = LimelightHelpers.getBotPose_wpiBlue("limelight-intake");
      // botpose_shooter = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");

      // if(botpose_intake[0] != 0){
      //   if(distanceFromSpeaker(botpose_intake[0],botpose_intake[1]) < 20){
      //     m_robotDrive.resetOdometry(new Pose2d(botpose_shooter[0], botpose_shooter[1], new Rotation2d(Math.toRadians(botpose_shooter[5]))));
      //   }
      // }
    }

    @Override
      public void end(boolean interrupted) {
    }

    private double distanceFromSpeaker(double x, double y) {
      double xDistance = x - Constants.TAG_4_X_POS;
      double yDistance = y - Constants.TAG_4_Y_POS;
      return Math.hypot(xDistance, yDistance);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }  
}
