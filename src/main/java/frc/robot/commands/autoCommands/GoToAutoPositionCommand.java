package frc.robot.commands.autoCommands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToAutoPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private PIDController xController;
  private PIDController yController;
  private PIDController rotateController;

  Pose2d m_goToPose;
  Supplier<Pose2d> m_goToPoseSupplier;

  double errorMargin = 0.02; 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToAutoPositionCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> goToPoseSupplier) {
    m_driveSubsystem = driveSubsystem;
    m_goToPoseSupplier = goToPoseSupplier;

    xController = new PIDController(0, 0,0); //Best values: 0.5, 0, 1.15
    yController = new PIDController(0, 0,0);
    rotateController = new PIDController(0, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goToPose = m_goToPoseSupplier.get();

    // if(DriverStation.getAlliance().get() = (DriverStation.Alliance.Red)){ 
      //   m_goToPose = GeometryUtil.flipFieldPose(m_goToPose);
      // }

    m_driveSubsystem.driveAutoSpeedFF(0, 0, 0, true, true);

    // xController = new PIDController(0.75, 0,0.05); //Best values: 0.5, 0, 1.15
    // yController = new PIDController(0.75, 0,0.05);
    // rotateController = new PIDController(0.01, 0.0, 0.0);

    // xController = new PIDController(3.5, 0,1.75); //Best values: 0.5, 0, 1.15
    // yController = new PIDController(0.75, 0,0.05);
    // rotateController = new PIDController(0.01, 0.0, 0.0);
    
    xController = new PIDController(1.1, 0,0.14); //Best values: 0.5, 0, 1.15
    yController = new PIDController(1.1, 0,0.14);
    rotateController = new PIDController(0.03, 0.0, 0.0);


    rotateController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // SmartDashboard.putNumber("Set Point X", m_goToPose.getX());
    // SmartDashboard.putNumber("Set Point Y", m_goToPose.getY());
    // SmartDashboard.putNumber("Set Point Rot", m_goToPose.getRotation().getDegrees());
    

    double botPoseX = m_driveSubsystem.getPose().getX();
    double botPoseY = m_driveSubsystem.getPose().getY();
    double botPoseRot = m_driveSubsystem.getPose().getRotation().getDegrees(); 
          
    m_driveSubsystem.driveAutoSpeedFF(
      xController.calculate(botPoseX, m_goToPose.getX()),
      yController.calculate(botPoseY, m_goToPose.getY()),
      rotateController.calculate(botPoseRot, m_goToPose.getRotation().getDegrees()),
      true, 
      true
    );    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.driveAutoSpeedFF(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // double botPoseX = m_driveSubsystem.getPose().getX();
    // double botPoseY = m_driveSubsystem.getPose().getY();
    // double botPoseRot = m_driveSubsystem.getWrappedHeading().getDegrees();
    
    // if(Math.abs(botPoseX -  m_goToPose.getX()) <= errorMargin && Math.abs(botPoseY - m_goToPose.getY()) <= errorMargin){
    // // if(Math.abs(botPoseX -  m_goToPose.getX()) <= errorMargin && Math.abs(botPoseY - m_goToPose.getY()) <= errorMargin && Math.abs(botPoseRot - m_goToPose.getRotation().getDegrees()) <= 3){
    //   return true;
    // }
    return false;
  }
}
