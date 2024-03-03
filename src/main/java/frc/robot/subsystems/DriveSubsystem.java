// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;





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

  // The gyro sensor
    //   private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.GyroCanId);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private AprilTagLimelightSubsystem m_limelightSubsystem;

  private final Field2d field = new Field2d();
 


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getWrappedHeading(),
      //m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(AprilTagLimelightSubsystem limelightSubsystem) {

    m_limelightSubsystem = limelightSubsystem;


    SmartDashboard.putData(field);

    m_gyro.reset();

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            // new PIDConstants(3.0, 0.0, 0.5),// Translation PID constants
            // new PIDConstants(1.6, 0.0, 0.0),// Translation PID constants
            
            new PIDConstants(3.8, 0.0,0.2),// Rotation PID constants
            new PIDConstants(0.01, 0.0, 0.0),// Rotation PID constants
            // new PIDConstants(0.0, 0.0, 0.0),
            5.5, // Max module speed, in m/s
            
            0.368, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),  
        () -> DriverStation.getAlliance().equals(DriverStation.Alliance.Red),
        this // Reference to this subsystem to set requirements
    );
}

public ChassisSpeeds getChassisSpeeds(){
  // SwerveModuleState[] swerveModuleStates = {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()};
  return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
}
private SwerveModuleState[] getModuleStates() {
  return new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState()
  };
}
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
   if (m_limelightSubsystem.getId() > 0){
    resetOdometry(getLimelightPose());
    // setGyroYaw(m_limelightSubsystem.getBotPose().getRotation().getDegrees()); //accuracy of april tag may be worse than gyro drift
   }else{
      m_odometry.update(
        //m_gyro.getRotation2d(),
        getWrappedHeading(),
        // new Rotation2d(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
         });
   }


  

    // SmartDashboard.putNumber("pose 2d rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("getWrappedHeading", getWrappedHeading().getDegrees());
    SmartDashboard.putNumber("pose X", getPose().getX());
    SmartDashboard.putNumber("pose Y", getPose().getY());

    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  // public Pose2d getPose(){
  //   return new Pose2d(
  //     new Translation2d(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY()),
  //     new Rotation2d(m_gyro.getYaw().getValueAsDouble())
  //   );
  // }

  public void setGyro(double degrees){
    m_gyro.setYaw(degrees);
    m_odometry.resetPosition(
      new Rotation2d(degrees), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition() 
      },
      new Pose2d(new Translation2d(0,0), new Rotation2d(degrees))
    );
  }
public void setGyroYaw(double yaw){
  m_gyro.setYaw(yaw);
}

public void setGyroYawUsingAprilTag(){

  if(m_limelightSubsystem.getId() > 0){
    m_gyro.setYaw(m_limelightSubsystem.getBotPose().getRotation().getDegrees());
  }
  
}
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        //m_gyro.getRotation2d(),
        getWrappedHeading(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  public Pose2d getLimelightPose(){
    return new Pose2d(m_limelightSubsystem.getBotPose().getX(), m_limelightSubsystem.getBotPose().getY(), getWrappedHeading() );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
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

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
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

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false, true);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  // public void setModuleStates(SwerveModuleState[] desiredStates) {
  //   SwerveDriveKinematics.desaturateWheelSpeeds(
  //       desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
  //   m_frontLeft.setDesiredState(desiredStates[0]);
  //   m_frontRight.setDesiredState(desiredStates[1]);
  //   m_rearLeft.setDesiredState(desiredStates[2]);
  //   m_rearRight.setDesiredState(desiredStates[3]);
  // 

  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  //   m_frontLeft.resetEncoders();
  //   m_rearLeft.resetEncoders();
  //   m_frontRight.resetEncoders();
  //   m_rearRight.resetEncoders();
  // }

  /** Zeroes the heading of the robot. */
  // public void zeroHeading() {
  //   m_gyro.reset();
  // }

  /**
   * Returns the heading of the robot.
   *
   */
  public Rotation2d getHeading() {
   // return m_gyro.getYaw().getValueAsDouble();
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));

  }

  public Rotation2d getWrappedHeading() {
   // return m_gyro.getYaw().getValueAsDouble();
    return Rotation2d.fromDegrees(MathUtil.inputModulus(m_gyro.getYaw().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0), -180.0, 180.0));

  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
//   public double getTurnRate() {
//     return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
//   }
}