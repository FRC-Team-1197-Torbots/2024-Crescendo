package frc.robot.Commands.Limelight;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;


public class ScanAprilTag extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Limelight m_Limelight;
  // private final DriveSubsystem m_robotDrive;
  private double[] botpose_intake;
  private double[] botpose_shooter;
  private double coord_x;
  private double coord_y;
  private double angle;
  private double xDistance;
  private double yDistance;
  Optional<Alliance> color = DriverStation.getAlliance();

  public ScanAprilTag(Limelight subsystem) {
      m_Limelight = subsystem;
      // m_robotDrive = drive;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      botpose_intake = LimelightHelpers.getBotPose_wpiBlue("limelight-intake");
      botpose_shooter = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");

      if(botpose_shooter.length > 0 && botpose_intake.length > 0) {
        if(botpose_shooter[0] != 0) { //check if shooter limelight sees anything
        coord_x = botpose_shooter[0];
        coord_y = botpose_shooter[1];
        angle = botpose_shooter[5];
        if(distanceFromSpeaker(coord_x, coord_y) <= Constants.maxSpeakerDistance){
          m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(180 + angle))));
          // m_robotDrive.setAngle(180+angle);
        }
        else if(botpose_intake[0] != 0) {
          coord_x = botpose_intake[0];
          coord_y = botpose_intake[1];
          angle = botpose_shooter[5];
          m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(180 + botpose_intake[5]))));
          }
        }
        else if(botpose_intake[0] != 0){ //if shooter limelight sees nothing, check intake limelight
          coord_x = botpose_intake[0];
          coord_y = botpose_intake[1];
         if(distanceFromSpeaker(coord_x, coord_y) <= Constants.maxSpeakerDistance){ //if distance from speaker is <= max distance
            m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(botpose_intake[5])))); //180 + botpose_intake[5]
          }
          else if(botpose_shooter[0] != 0){
            coord_x = botpose_shooter[0];
            coord_y = botpose_shooter[1];
            m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(botpose_shooter[5]))));
            
          }
        }
      } else {
        System.err.println("Couldn't find the limelights");
      }

      

      // m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(botpose_shooter[5]))));//botpose_shooter[5]
      //seannys soodocode
      //if we see from limelight, check distance. if distance is too large, dont update. if its good, update
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
      public void end(boolean interrupted) {
    }

    private double distanceFromSpeaker(double x, double y) {
      
      if (color.isPresent()) {
          if (color.get() == Alliance.Red) {
            xDistance = x - Constants.AprilTag4PosX;
            yDistance = y - Constants.AprilTag4PosY;        
          }
          if (color.get() == Alliance.Blue) {
              xDistance = x - Constants.AprilTag7PosX;
              yDistance = y - Constants.AprilTag7PosY;
          }
      }else{
        
      }
      
      return Math.hypot(xDistance, yDistance);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }  
}
