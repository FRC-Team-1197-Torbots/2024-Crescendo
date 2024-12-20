package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Commands.Amp.AmpIntake;
import frc.robot.Commands.Amp.AmpOuttake;
import frc.robot.Commands.Amp.AmpScore;
import frc.robot.Commands.Amp.SlowOuttake;
import frc.robot.Commands.Auto.AutoAlign;
import frc.robot.Commands.Auto.DriveForward;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.Shoot;
import frc.robot.Commands.LongCommands.IntakeSequence;
import frc.robot.Commands.LongCommands.ShuttleRevUp;
import frc.robot.Commands.LongCommands.SpeakerRevUp;
import frc.robot.subsystems.AmpRollers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GeneratePath;
import frc.robot.utils.LimelightHelpers;
import frc.robot.Commands.Shooter.RevShooter;

public class ButtonCommands {
    private  DriveSubsystem m_RobotDrive;
    private Intake m_Intake;
    private Shooter m_Shooter;
    public Arm m_Arm;
    public  Blinkin m_Blinkin;
    private  AmpRollers m_AmpRollers;
    private  Elevator m_Elevator;
    private RobotContainer m_Robot;
    private CommandXboxController m_DriveController;

    public ButtonCommands(RobotContainer robot, DriveSubsystem drive, Intake intake, Shooter shooter, Arm arm, Blinkin blinkin, AmpRollers rollers, Elevator elevator, CommandXboxController driveController) {
        m_RobotDrive = drive;
        m_Intake = intake;
        m_Shooter = shooter;
        m_Arm = arm;
        m_Blinkin = blinkin;
        m_AmpRollers = rollers;
        m_Elevator = elevator;
        m_Robot = robot;
        m_DriveController = driveController;
    }

    // write command as method
    public Command ampPass () {
        return Commands.sequence(
            new InstantCommand(() -> m_Shooter.setTargetRPM(ShooterConstants.IdleSpeed)),
            new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.AmpPos)),
            new WaitUntilCommand(m_Arm::onAmpTarget),
            new AmpIntake(m_AmpRollers, AmpRollerConstants.IntakeVoltage).alongWith(
            new Shoot(m_Intake)));
    } 
    
    public Command getNote() {
        return Commands.sequence(
            new AutoAlign(m_RobotDrive),
            driveForwardAndIntake()).until(m_Intake::gamePieceStored);
    }

    private Command driveForwardAndIntake() {
        return new ParallelRaceGroup(
            new DriveForward(m_RobotDrive, m_Intake, 0.2),
            new WaitUntilCommand(this::closeToNote).andThen(new IntakeSequence(m_Intake, m_Arm)));
    }

    private boolean closeToNote() {
        return LimelightHelpers.getTX("limelight-left") > -10;
    }
    
    public Command revUp() { 
        return new ConditionalCommand(
            new ShuttleRevUp(m_Shooter, m_RobotDrive, m_Arm, m_DriveController),
            new SpeakerRevUp(m_Shooter, m_RobotDrive, m_Arm), 
            m_Robot::inShuttleMode);
    }

    public Command AutorevUp() {
        return Commands.runOnce(() -> m_Shooter.setTargetRPM(ShooterConstants.ShootingRPM));
    }

    public Command revUpAndAim() {
        return Commands.sequence(
            new InstantCommand(() -> m_Arm.setTargetAngle(m_Arm.setAngleFromDistance())),
            new WaitUntilCommand(m_Shooter::onTarget), 
            new Shoot(m_Intake));
    }

    public Command ampPassBack() {
        return Commands.sequence(
            new InstantCommand(() -> m_Robot.setAmpMode(false)),
            new AmpOuttake(m_Shooter, m_AmpRollers),
            new SlowOuttake(m_Shooter),
            new RunIntake(m_Intake, IntakeConstants.IntakeSpeed));
    }

    public Command outtake() { 
        return Commands.parallel(
            new InstantCommand(() -> m_Robot.setAmpMode(false)),
            new AmpScore(m_AmpRollers, 4.0),
            new StartEndCommand( 
            () -> m_Intake.runIntake(IntakeConstants.OuttakeSpeed),
            () -> m_Intake.stopMotor(), m_Intake), 
            new StartEndCommand( 
            () -> m_Shooter.setTargetRPM(-ShooterConstants.IdleSpeed),
            () -> m_Shooter.stopMotor()));
    }

    public Command driveAndScore() {
        return Commands.sequence(
            GeneratePath.driveToPoint(m_RobotDrive, 1.55, 5.58),
            revUpAndShoot());
    }  
}

