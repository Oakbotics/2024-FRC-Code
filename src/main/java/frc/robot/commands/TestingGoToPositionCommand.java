// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;
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
public class TestingGoToPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private PIDController xController;
  private PIDController yController;
  private PIDController rotateController;

  Pose2d m_goToPose;

 
  double setPoseX = 1;
  double setPoseY = 1;
  double setPoseRot = 0;
  double errorMargin = 0.02; 

  double botPoseX;
  double botPoseY;
  double botPoseRot;
  // Pose2d ampPose = new Pose2d(ampPoseX, ampPoseY, Rotation2d.fromDegrees(ampPoseRot));
  // Pose2d ampPose;
  // Pose2d ampPoseBlue = new Pose2d(ampPoseXBlue, ampPoseY, Rotation2d.fromDegrees(ampPoseRot));
  // Pose2d ampPoseRed = GeometryUtil.flipFieldPose(ampPoseBlue);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestingGoToPositionCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
 
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);


    xController = new PIDController(0.0, 0,0.0); //Best values: 0.5, 0, 1.15
    yController = new PIDController(0.0, 0,0.0);
    rotateController = new PIDController(0.0, 0.0, 0.0);

    rotateController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    xController = new PIDController(0.75, 0,0.05); //Best values: 0.5, 0, 1.15
    yController = new PIDController(0.75, 0,0.05);
    rotateController = new PIDController(0.01, 0.0, 0.0);

    rotateController.enableContinuousInput(-180, 180);

    botPoseX = m_driveSubsystem.getPose().getX();
    botPoseY = m_driveSubsystem.getPose().getY();
    botPoseRot = m_driveSubsystem.getPose().getRotation().getDegrees(); 

    // setPoseX = botPoseX + 1;
    // setPoseY = botPoseY + 1;
    // setPoseRot = -45;

    // setPoseX = AutoConstants.bC3Pose.getX();
    // setPoseY = AutoConstants.bC3Pose.getY();
    // setPoseRot = 0;

    setPoseX = AutoConstants.bottomStartingPose.getX() + 2.3;
    setPoseY = AutoConstants.bottomStartingPose.getY() + 0.4;
    setPoseRot = 0;




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
    
    SmartDashboard.putNumber("Set Point X", setPoseX);
    SmartDashboard.putNumber("Set Point Y", setPoseY);
    SmartDashboard.putNumber("Set Point Rot", setPoseRot);
    

    botPoseX = m_driveSubsystem.getPose().getX();
    botPoseY = m_driveSubsystem.getPose().getY();
    botPoseRot = m_driveSubsystem.getPose().getRotation().getDegrees(); 
          
    m_driveSubsystem.drive(
      xController.calculate(botPoseX, setPoseX),
      yController.calculate(botPoseY, setPoseY),
      rotateController.calculate(botPoseRot, setPoseRot),
      true, 
      true
    );    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true, true);

    xController = new PIDController(0.0, 0,0.0); //Best values: 0.5, 0, 1.15
    yController = new PIDController(0.0, 0,0.0);
    rotateController = new PIDController(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees();
    
    if(Math.abs(botPoseX - setPoseX) <= errorMargin && Math.abs(botPoseY - setPoseY) <= errorMargin && Math.abs(botPoseRot - setPoseRot) <= 3){
      return true;
    }
    return false;
  }
}
