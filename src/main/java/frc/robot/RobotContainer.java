// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Arm.AutoArm;
import frc.robot.Commands.Arm.RunArm;
import frc.robot.Commands.Climber.RunClimber;
import frc.robot.Commands.Intake.AutoIntake;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.Shoot;
import frc.robot.Commands.Limelight.ScanAprilTag;
import frc.robot.Commands.Shooter.AmpShooter;
import frc.robot.Commands.Shooter.RevShooter;
import frc.robot.Commands.Shooter.ShootAuto;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ClimberConstants.ClimberDirection;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.time.Instant;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Auto Selectors
  private SendableChooser<String> positionChooser = new SendableChooser<>();

  private SendableChooser<String> autoNameChooser = new SendableChooser<>();
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter(m_Intake);
  private final Climber m_Climber = new Climber();
  public final Limelight m_Limelight = new Limelight(m_robotDrive);
  public final Arm m_Arm = new Arm(m_robotDrive, m_Limelight);

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_MechController = new CommandXboxController(1);
  private final Trigger exTrigger = new Trigger(m_robotDrive::checkLocked);
  private final Trigger beamTrigger = new Trigger(m_Intake::gamePieceStored);
  private final Trigger atShooterTarget = new Trigger(m_Shooter::onTarget);
  private final Trigger atAmpTarget = new Trigger(m_Shooter::ampOnTarget);
  private final Trigger atArmTarget = new Trigger(m_Arm::onTarget);
  private double speed = 0.7;
  private boolean inTeleop = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Add subsystems to different subsystems
    // m_Arm.getDriveSubsystem(m_robotDrive);

    // Configure the button bindings
    configureButtonBindings();

    // Add auto selector and commands used
    addAutoPaths();
    registerAutoCommands();
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
        true, true),
        m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    exTrigger.whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    // beamTrigger.onTrue(new InstantCommand(() -> m_Shooter.idleMotor(), m_Shooter));
    beamTrigger.onFalse(new InstantCommand(() -> m_Shooter.stopMotor(), m_Shooter));
    
    //Intake Routines
    m_driverController.rightTrigger(0.5).and(beamTrigger.negate()) //Runs Intake while running shooter backwards to prevent pieces from ejecting
      .whileTrue(
        new ParallelCommandGroup(
          new RunIntake(m_Intake, IntakeConstants.IntakeSpeed), 
          new RunArm(m_Arm, ArmConstants.IntakePos),
          new RunCommand(()-> m_Shooter.runShooter(-0.4))));
    
    // m_driverController.rightTrigger(0.5).and(beamTrigger.negate())//Runs Intake then backspins it for 0.2 seconds
    //   .whileTrue(
    //     new SequentialCommandGroup(
    //       new ParallelCommandGroup(
    //         new RunIntake(m_Intake, IntakeConstants.IntakeSpeed), 
    //         new RunArm(m_Arm, ArmConstants.IntakePos)),
    //       new RunCommand(() -> m_Intake.runIntake(-0.2)),
    //       new WaitCommand(-0.2),
    //       new InstantCommand(() -> m_Intake.stopMotor())));

    // m_driverController.rightBumper().whileTrue(new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.AmpPos)),
    //     new AmpShooter(m_Shooter)),
    //   new Shoot(m_Intake),
    //   new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos))));

    m_driverController.leftBumper().and(atShooterTarget).onTrue(new Shoot(m_Intake));

    // // limelight aiming
    // m_driverController.leftTrigger(0.5)
    //   .whileTrue(new SequentialCommandGroup(
    //     new ScanAprilTag(m_Limelight),
    //     // new AimRobot(m_robotDrive),
    //     new ParallelCommandGroup(
    //       new StartEndCommand(
    //         () -> m_Arm.setTargetAngle(m_Arm.setAngleFromDistance()),
    //         () -> m_Arm.setTargetAngle(ArmConstants.StorePos)),
    //       new RevShooter(m_Shooter))));

    m_driverController.leftTrigger(0.5)
      .whileTrue(new SequentialCommandGroup(
        new ParallelCommandGroup(
          new StartEndCommand(
            () -> m_Arm.setTargetAngle(ArmConstants.SubwooferPos), 
            () -> m_Arm.setTargetAngle(ArmConstants.StorePos)), 
          new RevShooter(m_Shooter))));

    //ANGLE TEST CODE
    // m_driverController.leftTrigger(0.5)
    // .whileTrue(new ParallelCommandGroup(
    //   new StartEndCommand(
    //     // () -> m_Arm.setStates(ArmStates.TEST),
    //     () -> m_Arm.setAngleFromDistance(m_robotDrive.distanceFromSpeaker()), 
    //     () -> m_Arm.setStates(ArmStates.STORE)), 
    //     new RevShooter(m_Shooter)));

    m_driverController.a().whileTrue(new StartEndCommand( 
      () -> m_Intake.runIntake(IntakeConstants.OuttakeSpeed),
      () -> m_Intake.stopMotor(),
      m_Intake));
      
    m_driverController.y().onTrue(new ScanAprilTag(m_Limelight));
      
    m_MechController.a()
      .whileTrue(
        new RunClimber(m_Climber, ClimberDirection.DOWN));
        
    m_MechController.y()
      .whileTrue(
        new RunClimber(m_Climber, ClimberDirection.UP));
          
    m_MechController.x().onTrue(new InstantCommand(() -> m_Arm.toggleIntake())); 
      
      
    // PID testing
    // m_MechController.povUp().onTrue(new InstantCommand(() -> m_robotDrive.incrementKp(0.01)));
    // m_MechController.povDown().onTrue(new InstantCommand(() -> m_robotDrive.incrementKp(-0.01)));
    // m_MechController.povLeft().onTrue(new InstantCommand(() -> m_robotDrive.incrementKp(-0.1)));
    // m_MechController.povRight().onTrue(new InstantCommand(() ->m_robotDrive.incrementKp(0.01)));
    
    // m_driverController.povUp().onTrue(new InstantCommand(() -> m_Arm.incrementAngle(0.5)));
    // m_driverController.povDown().onTrue(new InstantCommand(() -> m_Arm.incrementAngle(-0.5)));
    // m_driverController.povLeft().onTrue(new InstantCommand(() -> m_Arm.incrementAngle(-10)));
    // m_driverController.povRight().onTrue(new InstantCommand(() -> m_Arm.incrementAngle(10)));
    // m_MechController.povUp().onTrue(new InstantCommand(() -> m_Shooter.incrementrpm(10)));
    // m_MechController.povDown().onTrue(new InstantCommand(() -> m_Shooter.incrementrpm(-10)));
    // m_MechController.povLeft().onTrue(new InstantCommand(() -> m_Shooter.incrementbot(-0.025)));
    // m_MechController.povRight().onTrue(new InstantCommand(() -> m_Shooter.incrementbot(0.025)));
    // m_MechController.a().whileTrue(new StartEndCommand( 
    //   () -> m_Intake.TestIntake(),
    //   () -> m_Intake.stopMotor(),
    //   m_Intake));
    // m_MechController.rightTrigger(0.5).whileTrue(new RunCommand(() -> m_Arm.setStates(ArmStates.TEST)));
    }
    
    private void registerAutoCommands() {
      NamedCommands.registerCommand("Shooter Auto Sequence", new ShootAuto(m_Arm, m_Shooter));
      NamedCommands.registerCommand("Intake Sequence", new AutoIntake(m_Arm, m_Intake));
      NamedCommands.registerCommand("Arm to Store", new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos)));
    }
    
    private void addAutoPaths() {
      positionChooser.addOption("Top (AMP)", "Top");
      positionChooser.addOption("Middle (SPEAKER)", "Middle");
      positionChooser.addOption("Bottom (STATION)", "Bottom");
      
      autoNameChooser.addOption("3 Note", "3 Note");
      autoNameChooser.addOption("2 Note", "2 Note");
      autoNameChooser.addOption("1 Note", "1 Note");
      autoNameChooser.addOption("0 Note", "0 Note");
      
      SmartDashboard.putData("Positioning", positionChooser);
      SmartDashboard.putData("Auto Choice", autoNameChooser);
    }
      
      /**
       * 
       * Use this to pass the autonomous command to the main {@link Robot} class.
       *
       * @return the command to run in autonomous
       */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(autoNameChooser.getSelected() + " " + positionChooser.getSelected());  
  }

  public void teleopInit() {
    inTeleop = true;
    m_Arm.setMotorMode(IdleMode.kBrake);
    m_robotDrive.setMotorMode(IdleMode.kBrake);
    m_Intake.setMotorMode(IdleMode.kBrake);
    m_Shooter.setMotorMode(IdleMode.kBrake);
    // m_Shooter.setMotorMode(IdleMode.kCoast);
    //m_robotDrive.setAutoName(getAutonomousCommand().getName());

    //Get end point
    //conver to pose2d
    //m_robotDrive.resetOdometry();

    
  }
 
  public void disableInit() {
    m_Arm.setMotorMode(IdleMode.kCoast);
    m_robotDrive.setMotorMode(IdleMode.kCoast);
    m_Intake.setMotorMode(IdleMode.kCoast);
  }

  public void autoInit() {
    m_Shooter.setMotorMode(IdleMode.kBrake);
    m_robotDrive.resetGyro();
    m_Arm.resetArm();
    m_Arm.setAutoTargets(getAutonomousCommand().getName());
  }

  public ScanAprilTag getScanAprilTag() {
    return new ScanAprilTag(m_Limelight);
  }
}
