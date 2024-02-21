// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagLimelightSubsystem extends SubsystemBase {

  private NetworkTable m_limelightTable;

  private final Field2d m_field = new Field2d();

  /** Creates a new ExampleSubsystem. */
  public AprilTagLimelightSubsystem() {
    SmartDashboard.putData("Field", m_field);
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight-april");
  }

  public Pose2d getBotPose(){
    double[] botPoseArray = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[10]); 

    Pose2d botPose = new Pose2d(botPoseArray[0], botPoseArray[1], Rotation2d.fromDegrees(botPoseArray[5]));

    m_field.setRobotPose(botPose);   
    return botPose;
  }

  public int getId(){
    int tid = (int) m_limelightTable.getEntry("tid").getDouble(-1);
    return tid;
  }
  
  @Override
  public void periodic() {
    getBotPose();
    SmartDashboard.putNumber("limelight x value", getBotPose().getX());
    SmartDashboard.putNumber("limelight y value", getBotPose().getY());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}