// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANSparkMax m_topConveyorMotor;
  private final CANSparkMax m_bottomConveyorMotor;


  // private final LaserCan intakeSensor = new LaserCan(ConveyorConstants.topIntakeSensorCAN); 


  public ConveyorSubsystem() {

    m_topConveyorMotor = new CANSparkMax(ConveyorConstants.kTopConveyorMotorCANID, MotorType.kBrushless);
    m_bottomConveyorMotor = new CANSparkMax(ConveyorConstants.kBottomConveyorMotorCANID, MotorType.kBrushless);

    m_topConveyorMotor.restoreFactoryDefaults();
    m_bottomConveyorMotor.restoreFactoryDefaults();

    m_topConveyorMotor.setInverted(true);
    m_bottomConveyorMotor.setInverted(true);

    
    m_bottomConveyorMotor.setInverted(false);
    m_topConveyorMotor.setInverted(true);


    // intakeSensor = new LaserCan(8); //ID 8 is top, ID 7 is bottom

  }

  // public void runConveyorSpeed(double speed){
  //   m_topConveyorMotor.set(speed);
  //   m_bottomConveyorMotor.set(-speed);
  // }

  public void runConveyorSpeed(double speed){
    m_topConveyorMotor.setVoltage(speed);
    m_bottomConveyorMotor.setVoltage(-speed);
  }


  public boolean getTopSensorTriggered(){
    
    return false;
    // return intakeSensor.getMeasurement().distance_mm < 250;
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
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Top intake sensor", intakeSensor.getMeasurement().distance_mm); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
