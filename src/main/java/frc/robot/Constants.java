// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 2.0; // radians per second 1.2
    public static final double kMagnitudeSlewRate = 4.5; // percent per second (1 = 100%) 1.8 //3.4
    public static final double kRotationalSlewRate = 5.0; // percent per second (1 = 100%) 2.0//4.0

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25); //26.5
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25); //26.5
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI/2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = -Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI/2;

    // SPARK MAX CAN IDs
    // public static final int kFrontLeftDrivingCanId = 9; 
    // public static final int kRearLeftDrivingCanId = 7; 
    // public static final int kFrontRightDrivingCanId = 3; 
    // public static final int kRearRightDrivingCanId = 5;

    // public static final int kFrontLeftTurningCanId = 8;
    // public static final int kRearLeftTurningCanId = 6;
    // public static final int kFrontRightTurningCanId = 2;
    // public static final int kRearRightTurningCanId = 4;

    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 9;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.07438; // 76.2 mm
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1; 
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1; //initially 0.05
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ArmConstants{
    //Spark Max & Spark Flex Constants
    public static final int Motor1 = 13; // front
    public static final int Motor2 = 14; // back
    public static final int encoderChannelA = 7;
    public static final int encoderChannelB = 8;
    public static final double TICKS_PER_DEGREE = 2911.0 / 81.0;

    //Storing Position values
    // 66.2
    public static final double StorePos = 46; //Need to change these two values later
    public static final double IntakePos = 144;
    public static final double SpeakerPos = 114.2; //53
    public static final double SecondShotSpeaker = 105;
    public static final double TestPos = 106;
    public static final double AmpPos = 106.05;//106
    public static final double SubwooferPos = 119;
    
    // Auto arm angles
    public static final double[] TwoNoteMidTargets = {119.0, 101.5};
    public static final double[] ThreeNoteMidTargets = {4,5,5,4};
    public static final double[] OneNoteTopTargets = {107.367};
    public static final double[] OneNoteBottomTargets = {107.888};

    //Arm PID Constants
    public static final double Arm_kP = 0.1;//0.1//around 0.5 degrees of error
    public static final double Arm_kI = 0; //0.00001;
    public static final double Arm_kD = 0;

    //Arm Motion Profiling Constants
    public static final double MaxAngularVelo = 160;
    public static final double MaxAngularAccel = 120;


    // Arm aiming constants
    // // polynomial
    // public static final double A = 1.69;
    // public static final double B = -17.3;
    // public static final double C = 136;
    // // linear
    // public static final double M = -6.77;
    // public static final double B = 121;
    // logarithmic
    public static final double A = -20.3;
    public static final double B = 123;
  }

  public static final class ClimberConstants{
    //Climber Spark Flex IDs
    public static final int LeftClimberMotor = 17;
    public static final int RightClimberMotor = 16;

    //Climber Top position in Encoder Ticks
    public static final double LeftClimberTopPos = 172.2;
    public static final double RightClimberTopPos = 177.5;

    //Constant Climber Speed
    public static final double ClimberUpSpeed = 1;
    public static final double ClimberDownSpeed = -0.7;

    public static enum ClimberDirection {
      UP, DOWN
    }
  }

  public static final class IntakeConstants{
    public static final int IntakeMotor = 10;
    public static final double IntakeSpeed = 0.6;
    public static final double OuttakeSpeed = -0.9;

    //Digital Input Constants
    public static final int breakBeam = 0;
  }

  public static final class ShooterConstants {
    // SparkMax & Spark Flex Constants
    public static final int TopMotor = 12;
    public static final int BottomMotor = 11;

    public static final double IdleSpeed = -0.4;
    public static final double TargetRPM = -5200;

  }

  public static final double AprilTag4PosX = 16.579;  
  public static final double AprilTag4PosY = 5.548;
  public static final double AprilTag7PosX = -0.0381;//-1.5in
  public static final double AprilTag7PosY = 5.547868;//218.42in

  public static final double maxSpeakerDistance = 3.3;
}
