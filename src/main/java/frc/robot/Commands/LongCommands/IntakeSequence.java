package frc.robot.Commands.LongCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Commands.Arm.RunArm;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class IntakeSequence extends ParallelRaceGroup {
    
    public IntakeSequence(Intake intake, Arm arm) {
        addCommands( // super( works too
            new RunIntake(intake, IntakeConstants.IntakeSpeed), 
            new RunArm(arm, ArmConstants.IntakePos));
    }
}
