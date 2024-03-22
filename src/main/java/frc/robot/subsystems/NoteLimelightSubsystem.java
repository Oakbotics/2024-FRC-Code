// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class NoteLimelightSubsystem extends SubsystemBase {

  private NetworkTable m_limelightTable;

  private final Field2d m_field = new Field2d();

  /** Creates a new ExampleSubsystem. */
  public NoteLimelightSubsystem() {
    SmartDashboard.putData("Field", m_field);
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight-note");
    m_limelightTable.getEntry("pipeline").setNumber(0);

  }



  public Pose2d getNotePose(Pose2d robotPose){
    double tx = m_limelightTable.getEntry("tx").getDouble(-1); 
    double ty = m_limelightTable.getEntry("ty").getDouble(-1); 
    double tv = m_limelightTable.getEntry("tv").getDouble(-1); 
    

    // double forwardD = LimelightConstants.limelightHeight * Math.tan(Math.toRadians(90 - 30 - Math.abs(ty)));// ADD THIRTY ITS TILTED
    double forwardD = LimelightConstants.limelightHeight/Math.tan(Math.toRadians(30 - ty));
    
    double sideD = -forwardD * Math.tan(Math.toRadians(tx));
    forwardD += 0.5;

    if(tv != 1){
      forwardD = 0;
      sideD = 0;
    }

    // if(tx>=0){
    //   forwardD += LimelightConstants.crosshairDistance;
    // }
    // else{
    //   forwardD -= LimelightConstants.crosshairDistance;
    // }

    
    Transform2d relativeNotePose = new Transform2d(forwardD, sideD, Rotation2d.fromDegrees(-tx));

    SmartDashboard.putNumber("Note Distance X", forwardD);
    SmartDashboard.putNumber("Note Distance Y", sideD);
    // SmartDashboard.putNumber("90 - Math.abs(ty)", 90 - 30 - ty);
    
    return robotPose.plus(relativeNotePose);
  }
 
  @Override
  public void periodic() {
    double tx = m_limelightTable.getEntry("tx").getDouble(-1); 
    double ty = m_limelightTable.getEntry("ty").getDouble(-1);  

    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("tx", tx);


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}