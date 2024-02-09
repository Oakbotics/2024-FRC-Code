// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AprilTagGoToCoordinate extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private  final LimelightSubsystem m_LimelightSubsystem;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotateController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AprilTagGoToCoordinate(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_LimelightSubsystem = limelightSubsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, limelightSubsystem);

    //  xController = new PIDController(0.5, 0,1.15); //Best values: 0.5, 0, 1.15
    //  yController = new PIDController(0.5, 0,1.15);
    xController = new PIDController(0.5, 0,1.15); //Best values: 0.5, 0, 1.15
    yController = new PIDController(0, 0,0);
     rotateController = new PIDController(0.5, 0, 0.2);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double botPoseX = m_LimelightSubsystem.getBotPose().getX();// field relative
        double targetPoseX = m_LimelightSubsystem.getAprilTagDistance().getX() + 3;// relative to robot
        double botPoseY = m_LimelightSubsystem.getBotPose().getY();
        double botPoseRotate = m_LimelightSubsystem.getBotPose().getRotation().getRadians();
        
        double xSetPoint = 14.8;
        double ySetPoint = 4;
        double rotSetPoint = 180;
        double errorMargin = 0.05;

        if(Math.abs(botPoseX - xSetPoint) <= errorMargin){
          botPoseX = xSetPoint;
        }
          
      //  m_driveSubsystem.drive(xController.calculate(botPoseX, xSetPoint), yController.calculate(botPoseY, ySetPoint), rotateController.calculate(botPoseRotate, rotSetPoint), true, true);    
        m_driveSubsystem.drive(xController.calculate(botPoseX, xSetPoint), 0,0, true, true);

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {



}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}