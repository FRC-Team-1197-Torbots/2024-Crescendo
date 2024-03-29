package frc.robot.Commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightHelpers;
import frc.robot.subsystems.Limelight;


public class ScanAprilTagAuto extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Limelight m_Limelight;
  private double[] botpose_intake;
  private double[] botpose_shooter;
  private double coord_x;
  private double coord_y;
  private double xDistance;
  private double yDistance;
  //Optional<Alliance> color = DriverStation.getAlliance();

  public ScanAprilTagAuto(Limelight subsystem) {
      m_Limelight = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      //System.out.println("ScanAprilTag run!");
      
      botpose_intake = LimelightHelpers.getBotPose_wpiBlue("limelight-intake");
      botpose_shooter = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");

      if(botpose_intake.length > 0 && botpose_shooter.length > 0) {
        if(botpose_shooter[0] != 0){ //check if shooter limelight sees anything
          m_Limelight.setX(botpose_shooter[0]);
          m_Limelight.setY(botpose_shooter[1]);
        // if(distanceFromSpeaker(coord_x, coord_y) <= Constants.maxSpeakerDistance){
        //   m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(botpose_shooter[5]))));
        //   // }
        // else if(botpose_intake[0] != 0){
        //   coord_x = botpose_intake[0];
        //   coord_y = botpose_intake[1];
        //   m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(180 + botpose_intake[5]))));
        // }
        }
        else if(botpose_intake[0] != 0){ //if shooter limelight sees nothing, check intake limelight
          m_Limelight.setX(botpose_shooter[0]);
          m_Limelight.setY(botpose_shooter[1]);
          // if(distanceFromSpeaker(coord_x, coord_y) <= Constants.maxSpeakerDistance){ //if distance from speaker is <= max distance
            // m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(180 + botpose_intake[5])))); //180 + botpose_intake[5]
          // }
          // else if(botpose_shooter[0] != 0){
          //   coord_x = botpose_shooter[0];
          //   coord_y = botpose_shooter[1];
          //   m_Limelight.resetOdometry(new Pose2d(coord_x, coord_y, new Rotation2d(Math.toRadians(botpose_shooter[5]))));
          // }
        }
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
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }  
}
