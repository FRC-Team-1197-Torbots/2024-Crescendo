package frc.robot.Commands.LongCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Commands.Shooter.RevShooter;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class SpeakerRevUp extends ParallelCommandGroup {

    public SpeakerRevUp(Shooter shooter, DriveSubsystem robotDrive, Arm arm) {
        addCommands(
            new RunCommand(() -> robotDrive.aimRobotAtSpeaker(), robotDrive),
            new StartEndCommand(
            () -> arm.setTargetAngle(arm.setAngleFromDistance()),
            () -> arm.setTargetAngle(ArmConstants.StorePos)),
            new RevShooter(shooter, ShooterConstants.ShootingRPM));
    }
}
