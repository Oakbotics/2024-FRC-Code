// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANSparkMax m_topConveyorMotor;
  private final CANSparkMax m_bottomConveyorMotor;
  private LaserCan topIntakeSensor;
  private LaserCan bottomIntakeSensor;


  public ConveyorSubsystem() {

    m_topConveyorMotor = new CANSparkMax(ConveyorConstants.kTopConveyorMotorCANID, MotorType.kBrushless);
    m_bottomConveyorMotor = new CANSparkMax(ConveyorConstants.kBottomConveyorMotorCANID, MotorType.kBrushless);

    m_topConveyorMotor.restoreFactoryDefaults();
    m_bottomConveyorMotor.restoreFactoryDefaults();
    
    m_bottomConveyorMotor.setInverted(false);

    topIntakeSensor = new LaserCan(ConveyorConstants.kTopIntakeSensorCANID);
    try {
      topIntakeSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      topIntakeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8,8,4,4 ));
      topIntakeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      // TODO Auto-generated catch block
      // e.printStackTrace();
    }
    bottomIntakeSensor = new LaserCan(ConveyorConstants.kBottomIntakeSensorCANID);
    try {
      bottomIntakeSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      bottomIntakeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8,8,4,4));
      bottomIntakeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    

  }

  public void runConveyorSpeed(double speed){
    m_topConveyorMotor.set(speed);
    m_bottomConveyorMotor.set(speed);
  }

  public boolean getSensorTriggered(){
    return getSensorValue() >= 0 && getSensorValue() <= 200;
  }

  public boolean getBottomSensorTriggered(){
    return (getBottomSensorValue() >= 0 && getBottomSensorValue() <= 70) || (getBottomSensorValue() > 200 && getSensorValue() > 200) ;
  }

  public double getSensorValue(){
    LaserCan.Measurement measurement = topIntakeSensor.getMeasurement();
    if (measurement != null) {
        if(measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
          return measurement.distance_mm;
        }
    } 
    
    return 0.0;
  }

  public double getBottomSensorValue(){
    LaserCan.Measurement measurement = bottomIntakeSensor.getMeasurement();
    if (measurement != null) {
        if(measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
          return measurement.distance_mm;
        }
    } 
    
    return 0.0;
  }

  public boolean getNoteAligned(){
    return (getSensorValue() >= 45 && getSensorValue() <= 80 )|| getSensorValue() >= 200;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ConveyorMethodCommand() {


    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Bottom Lidar value", getBottomSensorValue());
    // SmartDashboard.putNumber("Top Lidar value", getSensorValue());

    // SmartDashboard.putBoolean("Lidar Trigger", getSensorTriggered());

    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
