package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

public class Limelight extends SubsystemBase{
    double pose_x;
    double pose_y;
    private double[] botpose1;
    //Optional<Alliance> color = DriverStation.getAlliance();

    private DriveSubsystem m_DriveSubsystem;
    private double limelight_x;
    private double limelight_y;
    public Limelight(DriveSubsystem drive){
        m_DriveSubsystem = drive;
    }

    @Override
    public void periodic(){
      SmartDashboard.putNumber("Distance from april tag", LimelightHelpers.getTX("limelight-shooter"));
        // botpose1 = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");
        // double[] botpose2 = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");

        // if(botpose1[0] != 0){
            
        // }
        // else if(botpose2[0] != 0){

        // }

        // SmartDashboard.putNumber("Botpose X", botpose1[0]);
        // SmartDashboard.putNumber("Botpose Y", botpose1[1]);
        

    }

    public void resetOdometry(Pose2d pose){
        m_DriveSubsystem.resetOdometry(pose);
    }
    public void setX(double value) {
        limelight_x = value;
    }
    public void setY(double value) {
        limelight_y = value;
    }

    private double xDistanceFromSpeaker() {
    if (!DriverStation.getAlliance().isEmpty()){
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return limelight_x - Constants.AprilTag4PosX;
      }
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return limelight_x - Constants.AprilTag7PosX;
      }
      return 0;
    } else
      return 0;
  }

  private double yDistanceFromSpeaker() {
    if (!DriverStation.getAlliance().isEmpty()){
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return limelight_y - Constants.AprilTag4PosY;
      }
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return limelight_y - Constants.AprilTag7PosY;
      }
      return 0;
    } else
      return 0;
  }
  

  public double distanceFromSpeaker() {
    return Math.hypot(xDistanceFromSpeaker(), yDistanceFromSpeaker());
  }
    

}