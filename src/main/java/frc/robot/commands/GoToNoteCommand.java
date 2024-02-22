// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteLimelightSubsystem;
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
public class GoToNoteCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private  final NoteLimelightSubsystem m_LimelightSubsystem;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotateController;
  private AprilTagFieldLayout fieldLayout;
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
  public GoToNoteCommand(DriveSubsystem driveSubsystem, NoteLimelightSubsystem limelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_LimelightSubsystem = limelightSubsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, limelightSubsystem);

    xController = new PIDController(0.55, 0,1.25); 
    yController = new PIDController(0.55, 0,1.25);
    rotateController = new PIDController(0.02, 0.0, 0.0);

    rotateController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees();
    Pose2d botPose = new Pose2d(botPoseX,botPoseY, Rotation2d.fromDegrees(botPoseRot));
    xSetPoint = m_LimelightSubsystem.getNotePose(botPose).getX();
    ySetPoint = m_LimelightSubsystem.getNotePose(botPose).getY();
    rotSetPoint = m_LimelightSubsystem.getNotePose(botPose).getRotation().getDegrees();

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees();

    SmartDashboard.putNumber("unwrapped rotation", botPoseRot);      
    SmartDashboard.putNumber("Set Point X", xSetPoint);
    SmartDashboard.putNumber("Set Point Y", ySetPoint);
    SmartDashboard.putNumber("Set Point Rot", rotSetPoint);
    SmartDashboard.putNumber("Setpoint from controller", rotateController.getSetpoint());

    rotSetPoint = -5;
    xSetPoint = 4.312;
    ySetPoint = -0.377;
    if(Math.abs(botPoseRot - rotSetPoint) > 1){
      m_driveSubsystem.drive(xController.calculate(botPoseX, botPoseX), yController.calculate(botPoseY, botPoseX),rotateController.calculate(botPoseRot, rotSetPoint), true, true);
    }
    // m_driveSubsystem.drive(0, 0,rotateController.calculate(botPoseRot, rotSetPoint), true, true);
    else if(Math.abs(botPoseRot - rotSetPoint) <= 1){
       m_driveSubsystem.drive(xController.calculate(botPoseX, xSetPoint), yController.calculate(botPoseY, ySetPoint),rotateController.calculate(botPoseRot, rotSetPoint), true, true);
    }
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