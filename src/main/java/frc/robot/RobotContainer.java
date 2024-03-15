// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.DoNothing;
import frc.robot.Commands.Arm.AutoArm;
import frc.robot.Commands.Arm.RunArm;
import frc.robot.Commands.Climber.RunClimber;
import frc.robot.Commands.Drive.AimAtSpeaker;
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
import frc.robot.Constants.BlinkinConstants;
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.sql.Driver;
import java.time.Instant;
import java.util.List;
import java.util.function.BooleanSupplier;

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
  public final Blinkin m_Blinkin = new Blinkin(m_Intake);

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_MechController = new CommandXboxController(1);
  private final Trigger exTrigger = new Trigger(m_robotDrive::checkLocked);
  private final Trigger speakerOnTarget = new Trigger(m_robotDrive::facingSpeaker);
  private final Trigger closeToSpeaker = new Trigger(m_robotDrive::closeToSpeaker);
  private final Trigger beamTrigger = new Trigger(m_Intake::gamePieceStored);
  private final Trigger atShooterTarget = new Trigger(m_Shooter::onTarget);
  private final Trigger atAmpTarget = new Trigger(m_Shooter::ampOnTarget);
  private final Trigger atArmTarget = new Trigger(m_Arm::onTarget);
  private final Trigger intakeFinished = new Trigger(m_Intake::finishedIntakeState);
  private double speed = 0.7;

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
        // m_robotDrive.setTurnRate(m_robotDrive.calcAngle()),
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
    beamTrigger.onTrue(new InstantCommand(() -> m_Blinkin.setColor(BlinkinConstants.Red), m_Blinkin));
    beamTrigger.onFalse(new InstantCommand(() -> m_Blinkin.setColor(BlinkinConstants.White), m_Blinkin));
    
    // Intake Routines
    m_driverController.rightTrigger(0.5).and(beamTrigger.negate()) //Runs Intake while running shooter backwards to prevent pieces from ejecting
      .whileTrue(
        new ParallelCommandGroup(
          new RunIntake(m_Intake, IntakeConstants.IntakeSpeed), 
          new RunArm(m_Arm, ArmConstants.IntakePos),
          new StartEndCommand(
            () -> m_Shooter.runShooter(-0.4), 
            () -> m_Shooter.stopMotor())));
          
    // m_driverController.rightTrigger(0.5).and(beamTrigger.negate())//Runs Intake then backspins it for 0.2 seconds
    //   .whileTrue(
    //     new SequentialCommandGroup(
    //       new ParallelCommandGroup(
    //         new RunIntake(m_Intake, IntakeConstants.IntakeSpeed), 
    //         new RunArm(m_Arm, ArmConstants.IntakePos),
    //         () -> m_Shooter.runShooter(-0.4))));
    
    // back drive
    // m_driverController.rightTrigger()
    //   .onTrue(
    //     new InstantCommand(() -> m_Intake.setFinishedIntake(false)));
    //       // new RunCommand(() -> m_Intake.runIntake(-0.2)),
    //       // new WaitCommand(0.5),
    //       // new InstantCommand(() -> m_Intake.stopMotor())));
    // intakeFinished.onTrue(
    //   new SequentialCommandGroup(
    //     new PrintCommand("null"),
    //     new RunCommand(() -> m_Intake.setIntakeVoltage(-5)).withTimeout(0.02),
    //     // new WaitCommand(0.5),
    //     new InstantCommand(() -> m_Intake.stopMotor())));
    
    // m_driverController.rightTrigger()
    //   .onFalse(
        // new SequentialCommandGroup(
          // new ParallelCommandGroup
          // (
          // new RunCommand(() -> ]\[
            // m_Intake.runIntake(-0.2)).withTimeout(0.5), 
          // new WaitCommand(0.5)),
          // new InstantCommand(() -> m_Intake.stopMotor())));
    // m_driverController.rightBumper().whileTrue(new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.AmpPos)),
    //     new AmpShooter(m_Shooter)),
    //   new Shoot(m_Intake),
    //   new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos))));

    
    // m_driverController.rightBumper().whileTrue(new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.AmpPos)),
    //     new AmpShooter(m_Shooter)),
    //   // new Shoot(m_Intake),
    //   new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos))));

      
    m_driverController.leftBumper().and(atShooterTarget).onTrue(
      new SequentialCommandGroup(
        new Shoot(m_Intake),
        new InstantCommand(() -> m_Blinkin.setColor(BlinkinConstants.White))));

    m_driverController.leftTrigger(0.5).whileTrue(
      new SequentialCommandGroup(
        new ScanAprilTag(m_Limelight),
        new AimAtSpeaker(m_robotDrive),
        new ScanAprilTag(m_Limelight).onlyIf(closeToSpeaker),
        new ParallelCommandGroup(
          new RunCommand(() -> m_robotDrive.aimRobot(),m_robotDrive).onlyIf(closeToSpeaker),
          // new Shoot(m_Intake).onlyIf(m_Shooter::onTarget),
          new StartEndCommand(
            () -> m_Arm.setTargetAngle(m_Arm.setAngleFromDistance()),
            () -> m_Arm.setTargetAngle(ArmConstants.StorePos)
          ),
          new RevShooter(m_Shooter)
        )
      )
    );
      
    /*
     * ,
        new ParallelCommandGroup(
          new RevShooter(m_Shooter),
          new Shoot(m_Intake).onlyIf(atShooterTarget)
        ) 
     */

    m_driverController.rightBumper()
      .whileTrue(new SequentialCommandGroup(
        new ParallelCommandGroup(
          new StartEndCommand(
            () -> m_Arm.setTargetAngle(ArmConstants.SubwooferPos), 
            () -> m_Arm.setTargetAngle(ArmConstants.StorePos)), 
          new RevShooter(m_Shooter))));

    // Manual arm control
    // m_driverController.x()
    //       .whileTrue(new SequentialCommandGroup(
    //         new ParallelCommandGroup(
    //           new StartEndCommand(
    //             () -> m_Arm.setTargetAngle(ArmConstants.SubwooferPos), //ArmConstants.SubwooferPos
    //             () -> m_Arm.setTargetAngle(ArmConstants.StorePos)), 
    //           new RevShooter(m_Shooter))));

    //ANGLE TEST CODE
    // m_driverController.leftTrigger(0.5)
    // .whileTrue(new ParallelCommandGroup(
    //   new StartEndCommand(
    //     // () -> m_Arm.setStates(ArmStates.TEST),
    //     () -> m_Arm.setAngleFromDistance(m_robotDrive.distanceFromSpeaker()), 
    //     () -> m_Arm.setStates(ArmStates.STORE)), 
    //     new RevShooter(m_Shooter)));
    
    m_driverController.b().whileTrue(new StartEndCommand( 
      () -> m_Intake.runIntake(IntakeConstants.OuttakeSpeed),
      () -> m_Intake.stopMotor(),
      m_Intake));
      
    m_driverController.x().onTrue(new ScanAprilTag(m_Limelight));
      
    m_driverController.a()
      .whileTrue(
        new RunClimber(m_Climber, ClimberDirection.DOWN));
        
    m_driverController.y()
      .whileTrue(
        new RunClimber(m_Climber, ClimberDirection.UP));

    m_MechController.x().onTrue(new InstantCommand(() -> m_Arm.toggleIntake())); 
      
    m_MechController.y().onTrue(new InstantCommand(() -> m_Blinkin.setColor(-0.43)));

    m_MechController.a().onTrue(new InstantCommand(() -> m_Blinkin.setColor(BlinkinConstants.White)));

    // PID testing
    // m_MechController.povUp().onTrue(new InstantCommand(() -> m_robotDrive.incrementKp(0.01)));
    // m_MechController.povDown().onTrue(new InstantCommand(() -> m_robotDrive.incrementKp(-0.01)));
    // m_MechController.povLeft().onTrue(new InstantCommand(() -> m_robotDrive.incrementKp(-0.1)));
    // m_MechController.povRight().onTrue(new InstantCommand(() ->m_robotDrive.incrementKp(0.01)));
    
    // m_driverController.povUp().onTrue(new InstantCommand(() -> m_Arm.incrementAngle(0.25)));
    // m_driverController.povDown().onTrue(new InstantCommand(() -> m_Arm.incrementAngle(-0.25)));
    // m_driverController.povLeft().onTrue(new InstantCommand(() -> m_Arm.incrementAngle(-10)));
    // m_driverController.povRight().onTrue(new InstantCommand(() -> m_Arm.incrementAngle(10)));
    // m_MechController.povUp().onTrue(new InstantCommand(() -> m_Shooter.incrementrpm(10)));
    // m_MechController.povDown().onTrue(new InstantCommand(() -> m_Shooter.incrementrpm(-10)));
    // m_MechController.povLeft().onTrue(new InstantCommand(() -> m_Shooter.incrementbot(-0.25)));
    // m_MechController.povRight().onTrue(new InstantCommand(() -> m_Shooter.incrementbot(0.25)));
    // m_MechController.a().whileTrue(new StartEndCommand( 
    //   () -> m_Intake.TestIntake(),
    //   () -> m_Intake.stopMotor(),
    //   m_Intake));
    // m_MechController.rightTrigger(0.5).whileTrue(new RunCommand(() -> m_Arm.setStates(ArmStates.TEST)));
    }
    
    private void registerAutoCommands() {
      NamedCommands.registerCommand("Shooter Auto Sequence", new ShootAuto(m_Arm, m_Shooter).withTimeout(5));
      // NamedCommands.registerCommand("Set Arm to Target", new AutoArm(m_Arm, m_Shooter).withTimeout(5));
      //NamedCommands.registerCommand("Shoot and Limelight Aim", new RunArm(m_Arm, 114.2).alongWith(new ShootAuto));
      NamedCommands.registerCommand("Intake Sequence", new AutoIntake(m_Arm, m_Intake).withTimeout(7));
      NamedCommands.registerCommand("Store", new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos)));
      NamedCommands.registerCommand("Auto End", new ParallelCommandGroup(
        new InstantCommand(() -> m_Arm.setTargetAngle(ArmConstants.StorePos)),
        new InstantCommand(() -> m_Shooter.stopMotor())));
    }
    
    private void addAutoPaths() {
      positionChooser.addOption("Top (AMP)", "Top");
      positionChooser.addOption("Top to Center", "Top to Center");
      positionChooser.addOption("Middle (SPEAKER)", "Middle");
      positionChooser.addOption("Bottom (STATION)", "Bottom");

      autoNameChooser.addOption("5 Note", "5 Note");
      autoNameChooser.addOption("4 Note", "4 Note");
      autoNameChooser.addOption("3 Note", "3 Note");
      autoNameChooser.addOption("2 Note", "2 Note");
      autoNameChooser.addOption("1 Note", "1 Note");
      autoNameChooser.addOption("0 Note", "0 Note");
      SmartDashboard.putData("Positioning", positionChooser);
      SmartDashboard.putData("Auto Choice", autoNameChooser);
      SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().get().toString());
    }
      
      /**
       * 
       * Use this to pass the autonomous command to the main {@link Robot} class.
       *
       * @return the command to run in autonomous
       */
  public Command getAutonomousCommand() {
    try{
      String autoName = autoNameChooser.getSelected() + " " + positionChooser.getSelected();
      if (autoName.contains("4 Note Middle")) 
        return new PathPlannerAuto(autoName + " " + DriverStation.getAlliance().get());
      else
        return new PathPlannerAuto(autoName);

    }
    catch(Exception e){
      return new PathPlannerAuto("0 Note Bottom");
    }
      
  }

  public void teleopInit() { 
    m_Blinkin.setColor(BlinkinConstants.White);
    m_Arm.setMotorMode(IdleMode.kBrake);
    m_Arm.setTargetAngle(ArmConstants.StorePos);
    m_robotDrive.setMotorMode(IdleMode.kBrake);
    m_Intake.setMotorMode(IdleMode.kBrake);
    m_Shooter.setMotorMode(IdleMode.kBrake);
    m_Shooter.stopMotor();

    // m_Shooter.setMotorMode(IdleMode.kCoast);
    //m_robotDrive.setAutoName(getAutonomousCommand().getName());

    //Get end point
    //conver to pose2d
    //m_robotDrive.resetOdometry();
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
    m_Arm.setAutoTargets(autoNameChooser.getSelected() + " " + positionChooser.getSelected());
    m_Shooter.setMotorMode(IdleMode.kBrake);
    m_Shooter.resetAutoShots();
    m_robotDrive.setAngle(m_robotDrive.getAutoStartingAngle(getAutonomousCommand().getName()));
  }

  public ScanAprilTag getScanAprilTag() {
    return new ScanAprilTag(m_Limelight);
  }
}
