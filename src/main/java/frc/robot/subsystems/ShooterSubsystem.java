// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANSparkMax m_rightShooterMotor;
  private final CANSparkMax m_leftShooterMotor;

  public ShooterSubsystem() {

    m_rightShooterMotor = new CANSparkMax(ConveyorConstants.kRightShooterMotorCANID, MotorType.kBrushless);
    m_leftShooterMotor = new CANSparkMax(ConveyorConstants.kLeftShooterMotorCANID, MotorType.kBrushless);

    m_rightShooterMotor.setSmartCurrentLimit(50);
    m_leftShooterMotor.setSmartCurrentLimit(50);

    m_leftShooterMotor.restoreFactoryDefaults();
    m_rightShooterMotor.restoreFactoryDefaults();    

    // m_leftShooterMotor.setInverted(true );
    // m_rightShooterMotor.setInverted(true  );

    m_leftShooterMotor.setIdleMode(IdleMode.kCoast);
    m_rightShooterMotor.setIdleMode(IdleMode.kCoast);   
    
    
  }

  public void runShooterSpeed(double speed){
    if(speed != 0){
      m_rightShooterMotor.setVoltage(-speed-2);
      m_leftShooterMotor.setVoltage(-speed);
    }
    else{
      m_rightShooterMotor.setVoltage(-speed);
      m_leftShooterMotor.setVoltage(-speed);
    }
    
  }

  public double getShooterVelocity(){

    return (m_rightShooterMotor.getEncoder().getVelocity() + m_leftShooterMotor.getEncoder().getVelocity())/2;

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ShooterMethodCommand() {


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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
