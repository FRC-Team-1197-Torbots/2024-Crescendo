package frc.robot.Commands.LongCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Commands.Shooter.RevShooter;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class SubwooferRevUp extends ParallelCommandGroup {
    
    public SubwooferRevUp(Arm arm, Shooter shooter) {
        addCommands(
            new StartEndCommand(
                () -> arm.setTargetAngle(ArmConstants.SubwooferPos), 
                () -> arm.setTargetAngle(ArmConstants.StorePos)), 
            new RevShooter(shooter, ShooterConstants.SubwooferRPM));
    }
}
