// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilTagLimelightSubsystem;

import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToAmpCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotateController;

 
  double ampPoseX = 1.85;
  double ampPoseY = 7.85;
  double ampPoseRot = 90;
  double errorMargin = 0.05; 
  Pose2d ampPose = new Pose2d(ampPoseX, ampPoseY, Rotation2d.fromDegrees(ampPoseRot));
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToAmpCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
 
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);


    xController = new PIDController(0.55, 0,1.25); //Best values: 0.5, 0, 1.15
    yController = new PIDController(0.55, 0,1.25);
    rotateController = new PIDController(0.01, 0.0, 0.0);

    rotateController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
      ampPose = GeometryUtil.flipFieldPose(ampPose);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    


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
