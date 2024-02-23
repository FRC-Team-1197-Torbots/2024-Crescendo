// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.Arm.AutoArm;
import frc.robot.Commands.Arm.ManualArm;
import frc.robot.Commands.Arm.RunArm;
import frc.robot.Commands.Climber.RunClimber;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.Shoot;
import frc.robot.Commands.Shooter.RevShooter;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.AutoConstants.AutoPosition;
import frc.robot.Constants.ClimberConstants.ClimberDirection;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.time.Instant;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase.IdleMode;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Auto Chooser
  private SendableChooser positionChooser = new SendableChooser<String>();

  private SendableChooser autoNameChooser = new SendableChooser<String>();

  //private SendableChooser autoChooser = new SendableChooser<PathPlannerAuto>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter();
  private final Arm m_Arm = new Arm();
  private final Climber m_Climber = new Climber();

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_MechController = new CommandXboxController(1);
  private final Trigger exTrigger = new Trigger(m_robotDrive::checkLocked);
  private final Trigger beamTrigger = new Trigger(m_Intake::gamePieceStored);
  
  //private final Trigger gamePieceStored = new Trigger(m_Shooter::breakBeamState);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    NamedCommands.registerCommand("IntakeDown",(
        new AutoArm(m_Arm, ArmStates.INTAKE)));
    NamedCommands.registerCommand("RunIntake", new RunIntake(m_Intake, IntakeConstants.IntakeSpeed));
    NamedCommands.registerCommand("Print Command", new PrintCommand("Hope this works"));
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
    //new JoystickButton(m_driverController, Button.kR1.value)
        //.whileTrue(new RunCommand(
           // () -> m_robotDrive.setX(),
           // m_robotDrive));
    exTrigger.whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // m_driverController.povUp()
    //   .whileTrue(new ManualArm(m_Arm,-0.2));
    // m_driverController.povDown()
    //   .whileTrue
    //     (new ManualArm(m_Arm,0.2));
    m_driverController.leftBumper()
    .onTrue(
      new InstantCommand(
        () -> m_robotDrive.setX(), m_robotDrive));
    m_driverController.rightTrigger(0.5).and(beamTrigger.negate())
    .whileTrue(
      new ParallelCommandGroup(
        new RunIntake(m_Intake, IntakeConstants.IntakeSpeed), 
        new RunArm(m_Arm, ArmStates.INTAKE)));
    m_driverController.leftTrigger(0.5)
    .whileTrue(
      new ParallelCommandGroup(
        new RunArm(m_Arm, ArmStates.TEST), 
        new RevShooter(m_Shooter)));
      m_driverController.leftBumper().whileTrue(new Shoot(m_Intake));
      m_driverController.a().whileTrue(new StartEndCommand(() -> m_Intake.runIntake(IntakeConstants.OuttakeSpeed),
       () -> m_Intake.stopMotor(),
        m_Intake));
      // m_driverController.a().onTrue(new InstantCommand(() -> m_Intake.runIntake(IntakeConstants.OuttakeSpeed), m_Intake));
      // m_driverController.a().onFalse(new InstantCommand(() -> m_Intake.stopMotor(), m_Intake));
        m_driverController.povUp().onTrue(new InstantCommand(() -> m_Arm.incrementKp(0.001)));
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_Arm.incrementKp(-0.001)));
    
    m_MechController.a()
    .whileTrue(
      new RunClimber(m_Climber, ClimberDirection.DOWN));// We should test this code first
      m_MechController.y()
      .whileTrue(
        new RunClimber(m_Climber, ClimberDirection.UP));
        m_MechController.x().onTrue(new InstantCommand(() -> m_Arm.toggleIntake())); // Probably Test this later, might need to add a new command class for this
    m_MechController.povUp().onTrue(new InstantCommand(() -> m_Arm.incrementKi(0.000001)));
    m_MechController.povDown().onTrue(new InstantCommand(() -> m_Arm.incrementKi(-0.000001)));
    m_MechController.povLeft().onTrue(new InstantCommand(() -> m_Arm.incrementKd(0.00001)));
    m_MechController.povRight().onTrue(new InstantCommand(() -> m_Arm.incrementKd(-0.00001)));
    
  
  }

  private void addAutoPaths(){
    positionChooser.addOption("Top (AMP)", "Top");
    positionChooser.addOption("Middle", "Middle");
    positionChooser.addOption("Bottom", "Bottom");

    autoNameChooser.addOption("4 Note", "4 Note");
    autoNameChooser.addOption("2 Note", "2 Note");

    //autoChooser.addOption("4 Note Auto", );
  }

  /* 
  private AutoPosition selecPosition(){
    String position = positionChooser.getSelected().toString();
    if(position.equals("TOP")){
      return AutoPosition.TOP;
    }else if(position.equals("MIDDLE")){
      return AutoPosition.MIDDLE;
    }else{
      return AutoPosition.BOTTOM;
    }
  }*/

  

  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(autoNameChooser.getSelected() + " " + positionChooser.getSelected());

    /* 
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    */
  }

  

  public void teleopInit(){
    m_Arm.setMotorMode(IdleMode.kBrake);
    m_robotDrive.setMotorMode(IdleMode.kBrake);
  }

  public void disableInit(){
    m_Arm.setMotorMode(IdleMode.kCoast);
    m_robotDrive.setMotorMode(IdleMode.kCoast);
  }
}
