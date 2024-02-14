// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToSpeakerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private  final LimelightSubsystem m_LimelightSubsystem;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotateController;
  private AprilTagFieldLayout fieldLayout;

  double xSetPoint = 0;
  double ySetPoint = 0;
  double rotSetPoint = 0;
  double errorMargin = 0.05;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @throws IOException 
   */
  public GoToSpeakerCommand(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_LimelightSubsystem = limelightSubsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, limelightSubsystem);

    //  xController = new PIDController(0.5, 0,1.15); //Best values: 0.5, 0, 1.15
    //  yController = new PIDController(0.5, 0,1.15);
    xController = new PIDController(0.5, 0,1.15); //Best values: 0.5, 0, 1.15
    yController = new PIDController(0.5, 0,1.15);
    // rotateController = new PIDController(0.5, 0, 0.2);
    rotateController = new PIDController(0.01, 0, 0);


    rotateController.enableContinuousInput(-180, 180);
    // rotateController = new PIDController(0, 0, 0);
      
     //fieldLayout = new AprilTagFieldLayout("./AprilTagLocation.json");
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speakerPoseX = 0.0;
    double speakerPoseY = 2.7;

    double radius = 2;

    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getPose().getRotation().getDegrees();    
  
    
    double botDistance = Math.sqrt(Math.pow(speakerPoseX-botPoseX, 2) + Math.pow(speakerPoseY-botPoseY, 2));  //Should we use current botpose or desired one?

    rotSetPoint = Math.toDegrees(Math.asin((botPoseY-speakerPoseY)/ botDistance)); 

    xSetPoint = speakerPoseX - (radius * (speakerPoseX-botPoseX))/botDistance;
    ySetPoint = speakerPoseY - (radius * (speakerPoseY-botPoseY))/botDistance;
    double errorMargin = 0.05;

  //   if(botAngle > 0 ){
  //     rotSetPoint = -180+botAngle;
  //   }
   
  //  else{
  //   rotSetPoint = 180 +  botAngle;
  // }
    //SmartDashboard.putNumber("SpeakerPose",speakerPose.getX());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getPose().getRotation().getDegrees();

    // if(botPoseRot < 0){
    //   botPoseRot = -botPoseRot;
    // }

    SmartDashboard.putNumber("unwrapped rotation", botPoseRot);
          
    SmartDashboard.putNumber("Set Point X", xSetPoint);
    SmartDashboard.putNumber("Set Point Y", ySetPoint);
    SmartDashboard.putNumber("Set Point Rot", rotSetPoint);
    //  m_driveSubsystem.drive(xController.calculate(botPoseX, xSetPoint), yController.calculate(botPoseY, ySetPoint), rotateController.calculate(botPoseRotate, rotSetPoint), true, true);    
    m_driveSubsystem.drive(xController.calculate(botPoseX, xSetPoint), yController.calculate(botPoseY, ySetPoint),rotateController.calculate(botPoseRot, rotSetPoint), true, true);

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
    double botPoseRot = m_driveSubsystem.getPose().getRotation().getDegrees();
    if(Math.abs(botPoseX - xSetPoint) <= errorMargin && Math.abs(botPoseY - ySetPoint) <= errorMargin && Math.abs(botPoseRot - rotSetPoint) <= 20*errorMargin){
      return true;
    }
    return false;
  }
}