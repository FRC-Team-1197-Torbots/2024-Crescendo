package frc.robot.utils;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class GeneratePath {

    public static Command driveToPoint(DriveSubsystem drive, double x, double y) {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Translation2d> waypoints = PathPlannerPath.bezierFromPoses(
        drive.getPose(),
        new Pose2d(2.8, 4.27 , Rotation2d.fromDegrees(0)),
        new Pose2d(x, y, Rotation2d.fromDegrees(0)));
    
        PathConstraints constraints = new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared, AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared); // The constraints for this path.
    
        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        new GoalEndState(0.0, Rotation2d.fromDegrees(0))); // Goal end state. You can set a holonomic rotation here.

        path.preventFlipping = true;

        return AutoBuilder.followPath(path).until(() -> drive.positionOnTarget(x,y));
    }
}
