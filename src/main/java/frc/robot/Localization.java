package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;

public class Localization {
    private SwerveDriveOdometry m_Odometry;

    public Localization () {
 
    }

    // instance varfiab;ess 
    // odometry object


    // methods
    public void initializeOdometry(SwerveModulePosition[] swerveModulePositions, double gyroAngle) {
        m_Odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(gyroAngle),
            swerveModulePositions,
            new Pose2d(0,0,new Rotation2d(Math.toRadians(gyroAngle))
        ));
    }

    public void resetOdometry(SwerveModulePosition[] swerveModulePositions, double gyroAngle, Pose2d autoPose) {
        m_Odometry.resetPosition(Rotation2d.fromDegrees(gyroAngle), swerveModulePositions, autoPose);
    }
    
    public void updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] swerveModulePositions) {
        m_Odometry.update(gyroAngle, swerveModulePositions);

    }


    public SwerveDriveOdometry getOdometry() {
        return m_Odometry;
    }
    // update(limeligfht odometry)

}
