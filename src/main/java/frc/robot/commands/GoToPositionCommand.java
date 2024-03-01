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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotateController;

 
  double ampPoseXBlue = 2;
  double ampPoseY = 2;
  double ampPoseRot = 180;
  double errorMargin = 0.02; 
  // Pose2d ampPose = new Pose2d(ampPoseX, ampPoseY, Rotation2d.fromDegrees(ampPoseRot));
  // Pose2d ampPose;
  // Pose2d ampPoseBlue = new Pose2d(ampPoseXBlue, ampPoseY, Rotation2d.fromDegrees(ampPoseRot));
  // Pose2d ampPoseRed = GeometryUtil.flipFieldPose(ampPoseBlue);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToPositionCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
 
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);


    xController = new PIDController(0.75, 0,0.05); //Best values: 0.5, 0, 1.15
    yController = new PIDController(0.75, 0,0.05);
    rotateController = new PIDController(0.01, 0.0, 0.0);

    rotateController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)){
    //   ampPose = ampPoseRed;
    // }
    // else{
    //   ampPose = ampPoseBlue;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("Set Point X", ampPoseXBlue);
    SmartDashboard.putNumber("Set Point Y", ampPoseY);
    SmartDashboard.putNumber("Set Point Rot", ampPoseRot);
    

    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getPose().getRotation().getDegrees(); 
          
    m_driveSubsystem.drive(
      xController.calculate(botPoseX, ampPoseXBlue),
      yController.calculate(botPoseY, ampPoseY),
      rotateController.calculate(botPoseRot, ampPoseRot),
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
    
    if(Math.abs(botPoseX - ampPoseXBlue) <= errorMargin && Math.abs(botPoseY - ampPoseY) <= errorMargin && Math.abs(botPoseRot - ampPoseRot) <= 1){
      return true;
    }
    return false;
  }
}
