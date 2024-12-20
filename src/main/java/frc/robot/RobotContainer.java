// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.Rumble;
import frc.robot.Commands.Arm.ZeroArm;
import frc.robot.Commands.Auto.AutoAlign;
import frc.robot.Commands.Auto.DriveForward;
import frc.robot.Commands.Auto.FindNote;
import frc.robot.Commands.Auto.StopDriving;
import frc.robot.Commands.Intake.AutoIntake;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.Shoot;
import frc.robot.Commands.LongCommands.AmpScoreSequence;
import frc.robot.Commands.LongCommands.IntakeSequence;
import frc.robot.Commands.LongCommands.SubwooferRevUp;
import frc.robot.Commands.Shooter.ShootAuto;
import frc.robot.subsystems.AmpRollers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GeneratePath;

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
  private final DriveSubsystem m_RobotDrive = new DriveSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter(m_Intake);
  // private final Climber m_Climber = new Climber();
  public final Arm m_Arm = new Arm(m_RobotDrive);
  public final Blinkin m_Blinkin = new Blinkin();
  private final AmpRollers m_AmpRollers = new AmpRollers();
  private final Elevator m_Elevator = new Elevator();
  
  
  // Driver controllers
  private CommandXboxController m_DriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_MechController = new CommandXboxController(1);
  
  private final ButtonCommands m_ButtonCommands = new ButtonCommands(this, m_RobotDrive, m_Intake, m_Shooter, m_Arm, m_Blinkin, m_AmpRollers, m_Elevator, m_DriverController);
  
  // Triggers
  private final Trigger exTrigger = new Trigger(m_RobotDrive::checkLocked);
  private final Trigger intakeBeamTrigger = new Trigger(m_Intake::gamePieceStored);
  private final Trigger ampBeamTrigger = new Trigger(m_AmpRollers::gamePieceStored);
  private final Trigger atShooterTarget = new Trigger(m_Shooter::onTarget);
  private final Trigger ampMode = new Trigger(this::inAmpMode);
  private final Trigger shuttleMode = new Trigger(this::inShuttleMode);


  private boolean shuttling = false;
  private boolean ampAfterIntake = false;
  private boolean victor;
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
    SmartDashboard.putBoolean("Amp Mode", inAmpMode());
    SmartDashboard.putBoolean("Shuttle Mode", inShuttleMode());
    // Configure default commands
     m_RobotDrive.setDefaultCommand(m_RobotDrive.driveWithController(m_DriverController));
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

    exTrigger.whileTrue(new RunCommand(() -> m_RobotDrive.setX(), m_RobotDrive));
    intakeBeamTrigger.onTrue(new InstantCommand(() -> m_Shooter.idleMotor(), m_Shooter));
    intakeBeamTrigger.onTrue(new Rumble(m_DriverController,0.35));
    intakeBeamTrigger.onFalse(new InstantCommand(() -> m_Shooter.stopMotor(), m_Shooter));
    intakeBeamTrigger.and(ampMode).whileTrue(m_ButtonCommands.ampPass());

    /* ************************* Driver Controls ******************** */ 
    // Intake
    m_DriverController.rightTrigger(0.5).and(intakeBeamTrigger.negate()).whileTrue(new IntakeSequence(m_Intake, m_Arm));


    // Drive to note and intake
    m_DriverController.x().whileTrue(m_ButtonCommands.getNote());

    // Rev Up or point at amp or shuttle revUp in shuttlemode
    m_DriverController.leftTrigger(0.5).whileTrue(new ConditionalCommand(
      m_RobotDrive.pointAtAmp(m_DriverController),
      m_ButtonCommands.revUp(),
      ampBeamTrigger.or(ampMode)));

    // Subwoofer rev up
    m_DriverController.rightBumper().whileTrue(new SubwooferRevUp(m_Arm, m_Shooter));

    // pass from amp to shooter
    m_DriverController.a().toggleOnTrue(m_ButtonCommands.ampPassBack());

    // outtake
    m_DriverController.b().whileTrue(m_ButtonCommands.outtake());

    // ShootCommand
    m_DriverController.leftBumper().toggleOnTrue(new WaitUntilCommand(atShooterTarget).andThen(new Shoot(m_Intake)).withTimeout(5));

    // Score in Amp
    m_DriverController.y().toggleOnTrue(new AmpScoreSequence(this, m_Elevator, m_AmpRollers, m_Arm));
    
    // Drive to random point
    m_DriverController.start().whileTrue(m_ButtonCommands.driveAndScore());

    m_DriverController.back().onTrue(Commands.runOnce(() -> toggleVictor()));


    /* ************************* Mech Controls ******************** */ 
    
    // zero gyro *press to reset field relative drive*
    m_MechController.povUp().onTrue(new InstantCommand(() -> m_RobotDrive.resetGyro()));  
    m_DriverController.povUp().onTrue(new InstantCommand(() -> m_RobotDrive.resetGyro()));  

    // Toggle shuttle mode
    m_MechController.b().onTrue(new InstantCommand(() -> toggleShuttleMode()));    

    // Amp pass
    m_MechController.x().onTrue(new InstantCommand(() -> toggleAmpMode()));    
    
    // Zero Arm
    m_MechController.povDown().onTrue(new ZeroArm(m_Arm));
    m_DriverController.povDown().onTrue(new ZeroArm(m_Arm));

    // update from smartdashboard
    m_MechController.rightBumper().onTrue(new InstantCommand(() -> m_RobotDrive.updateFromSmartDashboard()));

    // intake
    m_MechController.rightTrigger(0.5).whileTrue(new RunIntake(m_Intake, IntakeConstants.IntakeSpeed));

  }

  private void setBlinkinColor() {
    if(isVictor()) {
      m_Blinkin.setColor(BlinkinConstants.Violet);
    }
    else if(inAmpMode()) {
      if(m_AmpRollers.gamePieceStored())
        m_Blinkin.setColor(BlinkinConstants.Green);
      else 
        m_Blinkin.setColor(BlinkinConstants.Blue);
    } else if(inShuttleMode()) {
        if(m_Intake.gamePieceStored())
          m_Blinkin.setColor(BlinkinConstants.Red);
        else
           m_Blinkin.setColor(BlinkinConstants.White);
    } else {
      if(m_Intake.gamePieceStored())
          m_Blinkin.setColor(BlinkinConstants.Orange);
        else
           m_Blinkin.setColor(BlinkinConstants.Black);
    }
  }
  
  private boolean isVictor() {
    return victor;
  }

  private void toggleVictor() {
    victor = !victor;
  }

  private void toggleAmpMode() {
    ampAfterIntake = !ampAfterIntake;
    SmartDashboard.putBoolean("Amp Mode", inAmpMode());
  }

  public void setAmpMode(boolean value) {
    ampAfterIntake = value;
    SmartDashboard.putBoolean("Amp Mode", inAmpMode());
  }

  private boolean inAmpMode() {
    return ampAfterIntake;
  }

  private void toggleShuttleMode() {
    shuttling = !shuttling;
    SmartDashboard.putBoolean("Shuttle Mode", inShuttleMode());
  }

  public boolean inShuttleMode() {
    return shuttling;
  }
      
  private void registerAutoCommands() {
     
    NamedCommands.registerCommand("Drive to Speaker", GeneratePath.driveToPoint(m_RobotDrive, 1.8, 5.56));
    NamedCommands.registerCommand("Aim and Shoot", m_ButtonCommands.revUpAndShoot());
    NamedCommands.registerCommand("Get Note", m_ButtonCommands.getNote().withTimeout(2));
    NamedCommands.registerCommand("Amp Score", new AmpScoreSequence(null, m_Elevator, m_AmpRollers, m_Arm));
    NamedCommands.registerCommand("Toggle Amp Pass Mode", new InstantCommand(() -> setAmpMode(true)));
    NamedCommands.registerCommand("Stop Driving", new StopDriving(m_RobotDrive));
    NamedCommands.registerCommand("AutoAlign", new AutoAlign(m_RobotDrive));
    NamedCommands.registerCommand("Drive Forward", new DriveForward(m_RobotDrive, m_Intake, 0.3));
    NamedCommands.registerCommand("Shooter Auto Sequence", new ShootAuto(m_Arm, m_Shooter).withTimeout(5));
    NamedCommands.registerCommand("Intake Sequence", new AutoIntake(m_Arm, m_Intake).withTimeout(7));
    NamedCommands.registerCommand("Find Note", new FindNote(m_RobotDrive, 0.1));
    NamedCommands.registerCommand("Store", new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos)));
    NamedCommands.registerCommand("Auto End", new ParallelCommandGroup(
      new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos)),
      new InstantCommand(() -> m_Shooter.stopMotor())));
    }
    
  private void addAutoPaths() {
    positionChooser.onChange(oh -> updateAutoChooser());  // update auto chooser when position is changed
    positionChooser.addOption("Top (AMP)", "Top");
    positionChooser.addOption("AI", "AI");
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

  public void robotInit() {
    m_Blinkin.setColor(BlinkinConstants.White);
    SmartDashboard.putBoolean("Shuttle Mode", inShuttleMode());
  }

  public void teleopInit() { 
    m_Arm.setMotorMode(IdleMode.kBrake);
    m_RobotDrive.setMotorMode(IdleMode.kBrake);
    m_Intake.setMotorMode(IdleMode.kBrake);
    m_Shooter.setMotorMode(IdleMode.kBrake);
    m_Shooter.stopMotor();
    m_RobotDrive.getAlliance();
    m_RobotDrive.setAprilTagID();
  }

  public void teleopPeriodic() {
    m_RobotDrive.updatePoseFromVision();
    shuttling = m_RobotDrive.distanceFromSpeaker() > 3.5;
    setBlinkinColor();
  }
 
  public void disableInit() {
    m_Blinkin.setColor(BlinkinConstants.Orange);
    m_Arm.setMotorMode(IdleMode.kCoast);
    m_RobotDrive.setMotorMode(IdleMode.kCoast);
    m_Intake.setMotorMode(IdleMode.kCoast);
  }

  public void autoInit() {
    m_RobotDrive.setMotorMode(IdleMode.kBrake);
    m_Blinkin.setColor(BlinkinConstants.Pink);
    m_Arm.resetArm();
    m_Arm.setAutoTargets(getAutonomousCommand().getName());
    m_Shooter.setMotorMode(IdleMode.kBrake);
    m_Shooter.resetAutoShots();
    m_RobotDrive.setAngle(m_RobotDrive.getAutoStartingAngle(getAutonomousCommand().getName()));
    m_RobotDrive.getAlliance();
  }
}
