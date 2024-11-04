// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToAmpCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final PIDController xController = DriveConstants.xController;
  private final PIDController yController = DriveConstants.yController;
  private final PIDController rotateController = DriveConstants.rotController;

 
  double ampPoseXBlue = 1.85;
  double ampPoseY = 6.8;    //Bumpers should hit amp, not center of robot
  double ampPoseRot = -90;
  double errorMargin = 0.05; 
  // Pose2d ampPose = new Pose2d(ampPoseX, ampPoseY, Rotation2d.fromDegrees(ampPoseRot));
  Pose2d ampPose;
  Pose2d ampPoseBlue = new Pose2d(ampPoseXBlue, ampPoseY, Rotation2d.fromDegrees(ampPoseRot));
  Pose2d ampPoseRed = GeometryUtil.flipFieldPose(ampPoseBlue);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToAmpCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
 
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    rotateController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      ampPose = ampPoseRed;
    }
    else{
      ampPose = ampPoseBlue;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // SmartDashboard.putNumber("Set Point X", ampPose.getX());
    

    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getPose().getRotation().getDegrees(); 
          
    m_driveSubsystem.drive(
      xController.calculate(botPoseX, ampPose.getX()),
      yController.calculate(botPoseY, ampPose.getY()),
      rotateController.calculate(botPoseRot, ampPose.getRotation().getDegrees()), 
      true, 
      true
    );    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees();
    
    if(Math.abs(botPoseX - ampPose.getX()) <= errorMargin && Math.abs(botPoseY - ampPose.getY()) <= errorMargin && Math.abs(botPoseRot - ampPose.getRotation().getDegrees()) <= 1){
      return true;
    }
    return false;
  }
}
