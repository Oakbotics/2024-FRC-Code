// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class ConveyorConstants {
    public static final int kTopConveyorMotorCANID = 14;
    public static final int kBottomConveyorMotorCANID = 15;

    public static final int kRightShooterMotorCANID = 20;
    public static final int kLeftShooterMotorCANID = 21;

    public static final double revvedShooterRPM = 800;

    public static final int kTopIntakeSensorCANID = 5;
    public static final int kBottomIntakeSensorCANID = 25;
  }

  public static class ClimberConstants{

    public static final int kClimberMotorCANID = 22;
  }

  public static class CANdleConstants{
    public static final int CANdleCanID = 26;
  }

  public static class LimelightConstants {
    public static final double limelightHeight = 0.3302;
    public static final double crosshairDistance = 0.58; 
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kTeleopMaxSpeedMetersPerSecond = 5.5;
    public static final double kTeleopMaxAngularSpeed = 3.0 * Math.PI; // radians per second

    public static final double kAutoMaxSpeedMetersPerSecond = 5.5;
    public static final double kAutoMaxAngularSpeed = 3.0 * Math.PI; // radians per second

    public static final double kSpeedLimiter = 0.5;
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // public static final SwerveDriveKinematics kfieldRelative = kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(null, null))

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 12;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 2;
    
    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 6;

    public static final int GyroCanId = 4;
    
    public static final boolean kGyroReversed = false;

    public static final PIDController xController = new PIDController(0.75, 0,0.05);
    public static final PIDController yController = new PIDController(0.75, 0,0.05);
    public static final PIDController rotController = new PIDController(0.01, 0.0, 0.0);
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
    public static final double kWheelDiameterMeters = 0.0762;
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

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.05;
  }    

  public static final class AutoConstants {
    public static final Pose2d topStartingPose = new Pose2d(0.8, 6.675, Rotation2d.fromDegrees(60.0));
    public static final Pose2d middleStartingPose = new Pose2d(1.4, 5.6, Rotation2d.fromDegrees(0));
    public static final Pose2d bottomStartingPose = new Pose2d(0.8, 4.59, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d redBottomStartingPose = new Pose2d(16.09, 4.74, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d redM5GetOutOfWayPose = new Pose2d(12.46, 1.5, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d redM5LineupPose = new Pose2d(9, 1.5, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d redM5Pose = new Pose2d(8.27, 0.73, Rotation2d.fromDegrees(-130.0));

    public static final Pose2d redM4GetOutOfWayPose = new Pose2d(9.79, 0.98, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d redM4LineupPose = new Pose2d(9.04, 3.37, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d redM4Pose = new Pose2d(8.26, 2.41, Rotation2d.fromDegrees(-130.0));


    
    public static final Pose2d subwooferShootSafe = new Pose2d(1.25, 5.69, Rotation2d.fromDegrees(0));

    public static final Pose2d bC1Pose = new Pose2d(3.02, 7.3, Rotation2d.fromDegrees(0.0));
    public static final Pose2d bC2Pose = new Pose2d(3.15, 5.6, Rotation2d.fromDegrees(0.0));
    public static final Pose2d bC3Pose = new Pose2d(3.02, 4.03, Rotation2d.fromDegrees(0.0));
    
    public static final Pose2d rC1Pose = new Pose2d(3.02, 4.03, Rotation2d.fromDegrees(0.0));  //RED AMP
    public static final Pose2d rC3Pose = new Pose2d(3.02, 7.35, Rotation2d.fromDegrees(0.0));  //RED PODIUM

    public static final Pose2d redAmpNoteLineupPose = new Pose2d(2.27, 4.08, Rotation2d.fromDegrees(0.0)); //RED AMP
    public static final Pose2d redPodiumNoteLineupPose = new Pose2d(2.00, 7.3, Rotation2d.fromDegrees(0.0)); //RED PODIUM

    

    // public static final Pose2d bC1LineupPose = new Pose2d(2.52, 7.3, Rotation2d.fromDegrees(0.0));
    // public static final Pose2d bC3LineupPose = new Pose2d(2.52, 4.03, Rotation2d.fromDegrees(0.0));
    public static final Pose2d blueAmpNoteLineupPose = new Pose2d(2.27, 7.25, Rotation2d.fromDegrees(0.0)); // BLUE AMP
    public static final Pose2d bluePodiumNoteLineupPose = new Pose2d(2.00, 4.03, Rotation2d.fromDegrees(0.0)); // BLUE PODIUM

    public static final Pose2d topFarShootPose = new Pose2d(3.35, 6.5, Rotation2d.fromDegrees(25)); //set later
    public static final Pose2d middleFarShootPose = new Pose2d(3.63, 5.6, Rotation2d.fromDegrees(0.0)); //set later
    public static final Pose2d bottomFarShootPose = new Pose2d(3.35, 4.7, Rotation2d.fromDegrees(-25)); //set later
    
    public static final Pose2d bottomGetOutOfWay = new Pose2d(5.00, 5.6, Rotation2d.fromDegrees(0.0)); //set later
   
    public static final Pose2d blueMidGetOutOfWay = new Pose2d(2.6, 1.96, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d blueM1_5Pose = new Pose2d(8.45, 6.58, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d blueM5Pose = new Pose2d(8.5, 0.97, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d blueM4GetOutOfWayPose = new Pose2d(4.71, 1.11, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d blueM4LineupPose = new Pose2d(7.57, 3.38, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d redMidGetOutOfWay = new Pose2d(12.92, 2.15, Rotation2d.fromDegrees(-130.00));
    // public static final Pose2d redM1_5Pose = new Pose2d(8.27, 6.88, Rotation2d.fromDegrees(-60.0));
    // public static final Pose2d redM5Pose = new Pose2d(8.5, 0.97, Rotation2d.fromDegrees(-60.0));
    
  }
}
