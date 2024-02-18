// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 

  private AnalogInput intakeSensor; 
  private final CANSparkMax m_leftIntakeMotor;
  private final CANSparkMax m_rightIntakeMotor;

  public IntakeSubsystem() {

    m_leftIntakeMotor = new CANSparkMax(IntakeConstants.kLeftIntakeMotorCANID, MotorType.kBrushless);
    m_rightIntakeMotor = new CANSparkMax(IntakeConstants.kRightIntakeMotorCANID, MotorType.kBrushless);
    intakeSensor = new AnalogInput(IntakeConstants.intakeSensorID); 

    m_rightIntakeMotor.restoreFactoryDefaults();
    m_leftIntakeMotor.restoreFactoryDefaults();    

    m_rightIntakeMotor.setInverted(true );
    m_leftIntakeMotor.setInverted(true  );

    m_rightIntakeMotor.setIdleMode(IdleMode.kCoast);
    m_leftIntakeMotor.setIdleMode(IdleMode.kCoast);    
  }

  public void runIntakeSpeed(double speed){
    m_leftIntakeMotor.set(speed);
    m_rightIntakeMotor.set(speed);
  }

  public double getIntakeVelocity(){

    return (m_leftIntakeMotor.getEncoder().getVelocity() + m_rightIntakeMotor.getEncoder().getVelocity())/2;

  }
  public boolean getSensorTriggered(){
    
    return intakeSensor.getValue() < 3;
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}


