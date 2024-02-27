package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

public class Limelight extends SubsystemBase{
    double pose_x;
    double pose_y;
    private double[] botpose1;

    private DriveSubsystem m_DriveSubsystem;
    
    public Limelight(DriveSubsystem drive){
        m_DriveSubsystem = drive;
    }

    @Override
    public void periodic(){
        botpose1 = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");
        double[] botpose2 = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");

        // if(botpose1[0] != 0){
            
        // }
        // else if(botpose2[0] != 0){

        // }

        SmartDashboard.putNumber("Botpose X", botpose1[0]);
        SmartDashboard.putNumber("Botpose Y", botpose1[1]);
        

    }

    public void resetOdometry(Pose2d pose){
        m_DriveSubsystem.resetOdometry(pose);
    }
    

}