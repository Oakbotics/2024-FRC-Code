// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteLimelightSubsystem;
import frc.robot.Constants.DriveConstants;
import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToNoteCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private  final NoteLimelightSubsystem m_LimelightSubsystem;
  private final PIDController xController = DriveConstants.xController;
  private final PIDController yController = DriveConstants.yController;
  private final PIDController rotateController = DriveConstants.rotController;
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

    rotateController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   /*  double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees();
    Pose2d botPose = new Pose2d(botPoseX,botPoseY, Rotation2d.fromDegrees(botPoseRot));
    xSetPoint = m_LimelightSubsystem.getNotePose(botPose).getX();
    ySetPoint = m_LimelightSubsystem.getNotePose(botPose).getY();
    rotSetPoint = m_LimelightSubsystem.getNotePose(botPose).getRotation().getDegrees();*/

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


    // if(Math.abs(botPoseRot - rotSetPoint) > 2){
    //   m_driveSubsystem.drive(0, 0, rotateController.calculate(botPoseRot, rotSetPoint), true, true);
    // }
    // m_driveSubsystem.drive(0, 0,rotateController.calculate(botPoseRot, rotSetPoint), true, true);
    // else if(Math.abs(botPoseRot - rotSetPoint) <= 0.5){
       m_driveSubsystem.drive(xController.calculate(botPoseX, xSetPoint), yController.calculate(botPoseY, ySetPoint),rotateController.calculate(botPoseRot, rotSetPoint), true, true);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    // double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees();
    // if(Math.abs(botPoseX - xSetPoint) <= errorMargin && Math.abs(botPoseY - ySetPoint) <= errorMargin && Math.abs(botPoseRot - rotSetPoint) <= 1){
      
    if(Math.abs(botPoseX - xSetPoint) <= errorMargin && Math.abs(botPoseY - ySetPoint) <= errorMargin){  
      return true;
    }
    return false;
  }
}