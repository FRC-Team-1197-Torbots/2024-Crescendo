// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AmpRollerConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.ClimberConstants.ClimberDirection;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Commands.Kaiden;
import frc.robot.Commands.Amp.AmpIntake;
import frc.robot.Commands.Amp.AmpScore;
import frc.robot.Commands.Arm.RunArm;
import frc.robot.Commands.Arm.ZeroArm;
import frc.robot.Commands.Climber.RunClimber;
import frc.robot.Commands.Intake.AutoIntake;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.Shoot;
import frc.robot.Commands.Shooter.RevShooter;
import frc.robot.Commands.Shooter.ShootAuto;
import frc.robot.subsystems.AmpRollers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
  public final Arm m_Arm = new Arm(m_robotDrive);
  public final Blinkin m_Blinkin = new Blinkin();
  private final AmpRollers m_AmpRollers = new AmpRollers();
  private final Elevator m_Elevator = new Elevator();
  

  // Driver controllers
  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_MechController = new CommandXboxController(1);

  // Triggers
  private final Trigger exTrigger = new Trigger(m_robotDrive::checkLocked);
  private final Trigger intakeBeamTrigger = new Trigger(m_Intake::gamePieceStored);
  private final Trigger ampBeamTrigger = new Trigger(m_AmpRollers::gamePieceStored);
  private final Trigger atShooterTarget = new Trigger(m_Shooter::onTarget);
  private boolean shuttleMode = false;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Add subsystems to different subsystems

    // Configure the button bindings
    configureButtonBindings();

    // Add auto selector and commands used
    addAutoPaths();
    registerAutoCommands();

    SmartDashboard.putBoolean("Shuttle Mode", inShuttleMode());

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
   * Use this method to 
   * define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  
    exTrigger.whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    intakeBeamTrigger.onTrue(new InstantCommand(() -> m_Shooter.idleMotor(), m_Shooter));
    intakeBeamTrigger.onFalse(new InstantCommand(() -> m_Shooter.stopMotor(), m_Shooter));
    intakeBeamTrigger.or(ampBeamTrigger).onFalse(new InstantCommand(() -> m_Blinkin.setColor(BlinkinConstants.White), m_Blinkin));
    intakeBeamTrigger.onTrue(new InstantCommand(() -> m_Blinkin.setColor(BlinkinConstants.Red), m_Blinkin));
    ampBeamTrigger.onTrue(new InstantCommand(() -> m_Blinkin.setColor(BlinkinConstants.Green), m_Blinkin));

    // Intake Routines
    m_driverController.rightTrigger(0.5).and(intakeBeamTrigger.negate()) //Runs Intake while running shooter backwards to prevent pieces from ejecting
    .whileTrue(
      new ParallelCommandGroup(
          new RunIntake(m_Intake, IntakeConstants.IntakeSpeed), 
        new RunArm(m_Arm, ArmConstants.IntakePos))); //IntakePos
      
    Command ampScore = (new SequentialCommandGroup(
      new InstantCommand(() -> m_Elevator.setTargetPos(ElevatorConstants.AmpPos)),
      new WaitUntilCommand(m_Elevator::atAmpHeight),
      new AmpScore(m_AmpRollers, AmpRollerConstants.ScoreVoltage),
      new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos)),
      new Kaiden().withTimeout(0.6),
      new InstantCommand(() -> m_Elevator.setTargetPos(ElevatorConstants.StorePos))));

    // Command shootSpeaker = new Shoot(m_Intake).onlyIf(atShooterTarget).andThen(new RunCommand(() -> m_Intake.stopMotor(), m_Intake).withTimeout(5));
    Command shootSpeaker = new WaitUntilCommand(atShooterTarget).andThen(new Shoot(m_Intake)).withTimeout(5);
    
    //ShootCommand
    m_driverController.leftBumper().toggleOnTrue(new ConditionalCommand(shootSpeaker, ampScore, intakeBeamTrigger));

    Command speakerRev = new ParallelCommandGroup(
          new RunCommand(() -> m_robotDrive.aimRobotAtSpeaker(),m_robotDrive),
          new StartEndCommand(
            () -> m_Arm.setTargetAngle(m_Arm.setAngleFromDistance()),
            () -> m_Arm.setTargetAngle(ArmConstants.StorePos)
          ),
          new RevShooter(m_Shooter, ShooterConstants.ShootingRPM)
      );

    Command shuttleRev = new ParallelCommandGroup(
          new RunCommand(() -> m_robotDrive.aimRobotShuttle(),m_robotDrive),
          new StartEndCommand(
            () -> m_Arm.setTargetAngle(ArmConstants.ShuttleAngle),
            () -> m_Arm.setTargetAngle(ArmConstants.StorePos)
          ),
          new RevShooter(m_Shooter, ShooterConstants.ShuttleRPM)
      );

    Command revUp = new ConditionalCommand(shuttleRev, speakerRev, this::inShuttleMode);

    Command pointAtAmp = new RunCommand(
    () -> m_robotDrive.drive(
    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    m_robotDrive.getAmpRotationSpeed(), 
    true, true),
    m_robotDrive);
    
    //Rev Up
    m_driverController.leftTrigger(0.5).whileTrue(new ConditionalCommand(pointAtAmp, revUp, ampBeamTrigger)
    );

    //Subwoofer rev up
    m_driverController.rightBumper()
      .whileTrue(new SequentialCommandGroup(
        new ParallelCommandGroup(
          new StartEndCommand(
            () -> m_Arm.setTargetAngle(ArmConstants.SubwooferPos), 
            () -> m_Arm.setTargetAngle(ArmConstants.StorePos)), 
          new RevShooter(m_Shooter, ShooterConstants.SubwooferRPM))));

    // outtake
    m_driverController.b().whileTrue(new ParallelCommandGroup(
      new AmpScore(m_AmpRollers, 4.0),
      new StartEndCommand( 
      () -> m_Intake.runIntake(IntakeConstants.OuttakeSpeed),
      () -> m_Intake.stopMotor(), m_Intake), 
      new StartEndCommand( 
      () -> m_Shooter.setTargetRPM(-ShooterConstants.IdleSpeed),
      () -> m_Shooter.stopMotor())));  

  
    //Mech Controls

    //Climber Down
    m_MechController.a().whileTrue(new RunClimber(m_Climber, ClimberDirection.DOWN));

    //Climber Up
    m_MechController.y().whileTrue(new RunClimber(m_Climber, ClimberDirection.UP));
    
    // zero gyro *press to reset field relative drive*
    m_MechController.start().onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));  

    m_MechController.b().onTrue(new InstantCommand(() -> toggleShuttleMode()));
    
    // test code
    
    
    //Amp
    m_MechController.x().and(ampBeamTrigger.negate()).toggleOnTrue((new SequentialCommandGroup(
      new InstantCommand(() -> m_Shooter.setTargetRPM(ShooterConstants.IdleSpeed)),
      new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.AmpPos)),
      new WaitUntilCommand(m_Arm::onAmpTarget),
      new AmpIntake(m_AmpRollers, AmpRollerConstants.IntakeVoltage).alongWith(
      new Shoot(m_Intake)))));
        
    //Zero Arm
    m_MechController.back().onTrue(new ZeroArm(m_Arm));
  }

  private void toggleShuttleMode() {
    shuttleMode = !shuttleMode;
    SmartDashboard.putBoolean("Shuttle Mode", inShuttleMode());
  }

  public boolean inShuttleMode() {
    return shuttleMode;
  }
      
  private void registerAutoCommands() {
    NamedCommands.registerCommand("Shooter Auto Sequence", new ShootAuto(m_Arm, m_Shooter).withTimeout(5));
    NamedCommands.registerCommand("Intake Sequence", new AutoIntake(m_Arm, m_Intake).withTimeout(7));
    NamedCommands.registerCommand("Store", new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos)));
    NamedCommands.registerCommand("Auto End", new ParallelCommandGroup(
      new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos)),
      new InstantCommand(() -> m_Shooter.stopMotor())));
    }
    
  private void addAutoPaths() {
    positionChooser.onChange(oh -> updateAutoChooser());  // update auto chooser when position is changed
    positionChooser.addOption("Top (AMP)", "Top");
    positionChooser.addOption("Middle (SPEAKER)", "Middle");
    positionChooser.addOption("Bottom (STATION)", "Bottom");
    positionChooser.setDefaultOption("None Selected", "None");
    SmartDashboard.putData("Positioning", positionChooser);
    SmartDashboard.putData("Auto Choice", autoNameChooser);
  }
  
private void updateAutoChooser() {
  List<String> autoNames = AutoBuilder.getAllAutoNames();
  autoNameChooser.close();
  autoNameChooser = new SendableChooser<>();
  
  if (positionChooser.getSelected() == "None") {
    autoNameChooser.addOption("Do Nothing", "Nothing");
  } else {
    for (String name : autoNames)
      if (!name.contains("#") && name.contains(positionChooser.getSelected()))
        autoNameChooser.addOption(name, name);
  }
  SmartDashboard.putData("Auto Choice", autoNameChooser);
  }
  
  
  /**
   * 
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
  public Command getAutonomousCommand() {
    try{
      return new PathPlannerAuto(autoNameChooser.getSelected());
    }
    catch(Exception e){
      return new PathPlannerAuto("Nothing");
    }  
  }

  public void teleopInit() { 
    SmartDashboard.putBoolean("Shuttle Mode", inShuttleMode());
    m_Blinkin.setColor(BlinkinConstants.White);
    m_Arm.setMotorMode(IdleMode.kBrake);
    m_robotDrive.setMotorMode(IdleMode.kBrake);
    m_Intake.setMotorMode(IdleMode.kBrake);
    m_Shooter.setMotorMode(IdleMode.kBrake);
    m_Shooter.stopMotor();
    m_robotDrive.getAlliance();
    m_robotDrive.setAprilTagID();
  }

  public void teleopPeriodic() {
    m_robotDrive.updatePoseFromVision();
  }
 
  public void disableInit() {
    m_Blinkin.setColor(BlinkinConstants.Orange);
    m_Arm.setMotorMode(IdleMode.kCoast);
    m_robotDrive.setMotorMode(IdleMode.kCoast);
    m_Intake.setMotorMode(IdleMode.kCoast);
  }

  public void autoInit() {
    m_Blinkin.setColor(BlinkinConstants.Pink);
    m_Arm.resetArm();
    m_Arm.setAutoTargets(getAutonomousCommand().getName());
    m_Shooter.setMotorMode(IdleMode.kBrake);
    m_Shooter.resetAutoShots();
    m_robotDrive.setAngle(m_robotDrive.getAutoStartingAngle(getAutonomousCommand().getName()));
    m_robotDrive.getAlliance();
  }

}
