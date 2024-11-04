// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AprilTagLimelightSubsystem;

import java.io.IOException;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToSpeakerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
 
  private final PIDController xController = DriveConstants.xController;
  private final PIDController yController = DriveConstants.yController;
  private final PIDController rotateController = DriveConstants.rotController;
  private AprilTagFieldLayout fieldLayout;
  double speakerPoseX = 0.0; // X postion of Speaker Apriltag on blue side
  double speakerPoseY = 5.6; // Y postion of Speaker Apriltag on blue side
  Pose2d speakerPose = new Pose2d(speakerPoseX, speakerPoseY, Rotation2d.fromDegrees(0));
  double xSetPoint;
  double ySetPoint;
  double rotSetPoint;
  double errorMargin = 0.1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @throws IOException 
   */
  public GoToSpeakerCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
  
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    rotateController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double radius = 3.08; //setpoint distance from middle of speaker
    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    
    // Flips Pose of Speaker depending on Alliance
    if(DriverStation.getAlliance().get() == (DriverStation.Alliance.Red)){ 
      speakerPose = GeometryUtil.flipFieldPose(speakerPose);
    }
  
    double botDistance = Math.sqrt(
      Math.pow(speakerPose.getX() - botPoseX, 2) 
      + Math.pow(speakerPose.getY() - botPoseY, 2)
    );  

    xSetPoint = speakerPose.getX() - (radius * (speakerPose.getX() - botPoseX)) / botDistance;
    ySetPoint = speakerPose.getY() - (radius * (speakerPose.getY() - botPoseY)) / botDistance;

    rotSetPoint = Math.toDegrees(Math.asin( (botPoseY-speakerPose.getY()) / botDistance)); 
    
    
    // Flips rotation setpoint if on red allience because math
    if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){ 
      rotSetPoint = -rotSetPoint;
    }

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees();

    // SmartDashboard.putNumber("unwrapped rotation", botPoseRot);      
    // SmartDashboard.putNumber("Set Point X", xSetPoint);
    // SmartDashboard.putNumber("Set Point Y", ySetPoint);
    // SmartDashboard.putNumber("Set Point Rot", rotSetPoint);
    // SmartDashboard.putNumber("Setpoint from controller", rotateController.getSetpoint());

    m_driveSubsystem.drive(xController.calculate(botPoseX, xSetPoint), yController.calculate(botPoseY, ySetPoint),rotateController.calculate(botPoseRot, rotSetPoint), true, true);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees()
    ;
    if(Math.abs(botPoseX - xSetPoint) <= errorMargin && Math.abs(botPoseY - ySetPoint) <= errorMargin && Math.abs(botPoseRot - rotSetPoint) <= 1){
      return true;
    }
    return false;
  }
}