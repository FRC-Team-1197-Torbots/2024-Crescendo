package frc.robot.Commands.LongCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Shooter.RevShooter;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class ShuttleRevUp extends ParallelCommandGroup {

    public ShuttleRevUp(Shooter shooter, DriveSubsystem robotDrive, Arm arm, CommandXboxController controller) {
        addCommands(
            robotDrive.shuttleAimCommand(controller),
            new StartEndCommand(
            () -> arm.setTargetAngle(ArmConstants.ShuttleAngle),
            () -> arm.setTargetAngle(ArmConstants.StorePos)),
            new RevShooter(shooter, ShooterConstants.ShuttleRPM));
    }
}
