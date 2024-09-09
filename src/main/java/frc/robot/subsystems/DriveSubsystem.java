// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

//import com.ctre.phoenix
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.LimelightHelpers;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private MAXSwerveModule[] modules = new MAXSwerveModule[] {
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight
  };

  /*
   * private enum isLocked {
   * LOCK, UNLOCK
   * }
   */

  // The gyro sensor
  public final Pigeon2 m_gyro = new Pigeon2(15); // might have to change to pigeon

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private PIDController m_PidController;
  private double angleDelta;
  private double turningKp = 0.02;
  private double turningKd = 0.001;

  // Odometry class for tracking robot pose
  double[] botpose_shooter = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");
  SwerveDriveOdometry m_odometry;

  private SwerveDriveKinematics kinematics;
  // private isLocked m_isLocked = isLocked.UNLOCK;
  private boolean is_Locked = false;

  /* Field 2D on Robot Sim */
  private Field2d m_field2d = new Field2d();
  private double odometry_x;
  private double odometry_y;
  private boolean isRedAlliance;
  Optional<Alliance> color = DriverStation.getAlliance();
  private String m_autoName = "0 Note Bottom";

  // Robot's Unit Vector

  // Vector from robot's position to speaker

  // Coordinates of robot
  private double[] m_RobotCoords = new double[2];

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //m_autoName = autoName;
    //SmartDashboard.putNumber("Auto initial", PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().getDegrees());
    

    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(gyroWithOffset()),//-m_gyro.getAngle()
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }
      , PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName)
      //new Pose2d(0,0,new Rotation2d(Math.toRadians(0)))
    );


    m_RobotCoords[0] = m_odometry.getPoseMeters().getX();
    m_RobotCoords[1] = m_odometry.getPoseMeters().getY();

    m_PidController = new PIDController(turningKp, 0, turningKd);
    resetEncoders();
   
    // All other subsystem initialization
    // ...
    
            // All other subsystem initialization
        // ...

    kinematics = DriveConstants.kDriveKinematics;

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.003), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.772, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public double getAutoStartingAngle(String autoName) {
    m_autoName = autoName;
     if (!DriverStation.getAlliance().isEmpty()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        // SmartDashboard.putNumber("Red angle", PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().getDegrees());
        return 180 + GeometryUtil.flipFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName)).getRotation().getDegrees();
      } else {
        // SmartDashboard.putNumber("Blue angle", 180 + PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().getDegrees());
        return PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().getDegrees();
  
      }
    } else {
      return 0;
    }
  }

  public void getAlliance() {
    color = DriverStation.getAlliance();
  }

  public double gyroWithOffset() {

    if (!DriverStation.getAlliance().isEmpty()) {
      double offset;
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        offset = GeometryUtil.flipFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName)).getRotation().getDegrees();
        // if(offset == 180){
        //   offset = 0;
        // }
      } else {
        offset = PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().getDegrees();
      }
      return (-m_gyro.getAngle() + 0);
    } else {
      return 0;
    }
    
  }
  

  @Override
  public void periodic() {
    // SmartDashboard.putString("Auto Name", m_autoName);
    // SmartDashboard.putNumber("Angle from speaker", calcAngle());
    m_odometry.update(new Rotation2d(Math.toRadians(gyroWithOffset())), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
    });

    odometry_x = m_odometry.getPoseMeters().getX();
    odometry_y = m_odometry.getPoseMeters().getY();
    m_RobotCoords[0] = odometry_x;
    m_RobotCoords[1] = odometry_y;

    SmartDashboard.putNumber("Odometry X", odometry_x);
    SmartDashboard.putNumber("Odometry Y", odometry_y);

    SmartDashboard.putData("Robot Field", m_field2d);
    // SmartDashboard.putBoolean("Period", facingSpeaker());
    SmartDashboard.putNumber("Distance From Speaker (hypot)", distanceFromSpeaker());
    SmartDashboard.putNumber("Distance From Speaker (x)", Math.abs(getPose().getTranslation().getX() - getAprilTagPos().getX()));
    m_field2d.setRobotPose(m_odometry.getPoseMeters());


    // SmartDashboard.putNumber("Front Left Pos", m_frontLeft.getPosition().distanceMeters);
    // SmartDashboard.putNumber("Front Right Pos", m_frontRight.getPosition().distanceMeters);
    // SmartDashboard.putNumber("Back Left Pos", m_rearLeft.getPosition().distanceMeters);
    // SmartDashboard.putNumber("Back Right Pos", m_rearRight.getPosition().distanceMeters);
    // SmartDashboard.putNumber("Robot angular velocity", getSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(gyroWithOffset()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void resetGyro(){
    m_gyro.reset();
  }
  
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(targetStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * 
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(gyroWithOffset())) // angle flipped intentionally
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    // changeState();
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setZero() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }


  public void changeLockState() {
    is_Locked = !is_Locked;
  }

  public boolean checkLocked() {
    return !is_Locked;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void setAngle(double angle) {
    m_gyro.setYaw(angle);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyroWithOffset()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setMotorMode(IdleMode mode) {
    for (MAXSwerveModule module : modules) {
      module.setMotorMode(mode);
    } 
  }

  public void initializeOdometry(){
    m_odometry.resetPosition(Rotation2d.fromDegrees(45),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(botpose_shooter[0],
        botpose_shooter[1],
        new Rotation2d(Math.toRadians(90))));//180 + botpose_shooter[5]
  }

  private Translation2d getAprilTagPos() {
    if (color.isPresent()) {
      if (color.get() == Alliance.Red) {
        return Constants.redSpeakerCoords;
      }
      if (color.get() == Alliance.Blue) {
        return Constants.blueSpeakerCoords;
      }
      return new Translation2d();
    } else
      return new Translation2d();
  }
  public void setAutoName(String autoname){
    m_autoName = autoname;
    SmartDashboard.putString("Auto Name", m_autoName);
  }
  public double distanceFromSpeaker() {
    return getPose().getTranslation().getDistance(getAprilTagPos());
  }

  public double calcAngle() {
    Translation2d robotPos = getPose().getTranslation();
    Translation2d deltaPos = getAprilTagPos().minus(robotPos);
    double deltaAngle = (getPose().getRotation().getDegrees() - Math.toDegrees(Math.atan(deltaPos.getY() / deltaPos.getX())));
    if (color.isPresent())
      if (color.get() == Alliance.Red)
        deltaAngle -= 180;
    deltaAngle %= 360;

    if (deltaAngle > 180)
      return deltaAngle - 360;
    else if (deltaAngle < -180)
      return deltaAngle + 360;
    else
      return deltaAngle;


    // return -1 * Math.toDegrees(Math.atan(yDistanceFromSpeaker() / xDistanceFromSpeaker())); //maybe pi wuld help
  }

  public void aimRobot() {
    angleDelta = calcAngle();
    drive(0,0, setTurnRate(angleDelta),false,false);
  }
  
  public boolean facingSpeaker() {
    return Math.abs(calcAngle()) < 8;
  }

  public boolean closeToSpeaker() {
    return distanceFromSpeaker() < 3.1;
  }

  public double setTurnRate(double error) {
    return m_PidController.calculate(error);
  }

  public void incrementKp(double amount) {
    turningKp += amount;
    m_PidController.setP(turningKp);
    SmartDashboard.putNumber("Turning Kp", turningKp);
  }

  public void incrementKd(double amount) {
    turningKd += amount;
    m_PidController.setD(turningKp);
    SmartDashboard.putNumber("Turning Kd", turningKd);
  }

}
