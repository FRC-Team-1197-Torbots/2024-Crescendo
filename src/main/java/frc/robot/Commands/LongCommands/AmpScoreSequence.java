package frc.robot.Commands.LongCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Kaiden;
import frc.robot.Commands.Amp.AmpScore;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AmpRollers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class AmpScoreSequence extends SequentialCommandGroup {
    
    public AmpScoreSequence(RobotContainer robot, Elevator elevator, AmpRollers rollers, Arm arm) {
        addCommands(
            new InstantCommand(() -> robot.setAmpMode(false)),
            new InstantCommand(() -> elevator.setTargetPos(ElevatorConstants.AmpPos)),
            new WaitUntilCommand(elevator::atAmpHeight),
            new AmpScore(rollers, AmpRollerConstants.ScoreVoltage),
            new InstantCommand(() -> arm.setTargetAngle(ArmConstants.StorePos)),
            new Kaiden().withTimeout(0.3),
            new InstantCommand(() -> elevator.setTargetPos(ElevatorConstants.StorePos)));
    }
}
