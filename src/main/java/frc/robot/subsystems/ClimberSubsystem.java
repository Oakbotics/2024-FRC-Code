// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {
  public final CANSparkMax m_climberMotor;
  public final RelativeEncoder m_encoder;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    m_climberMotor = new CANSparkMax(ClimberConstants.kClimberMotorCANID, MotorType.kBrushless);
    m_encoder = m_climberMotor.getEncoder();
    m_climberMotor.restoreFactoryDefaults();

    m_climberMotor.setInverted(true);
    
  }

  public void setClimberSpeed(double speed){

    m_climberMotor.set(speed);

  }

  public double getEncoderValue(){

    return m_encoder.getPosition();

  }

  /**
   * Example command factory method.
   *
   * @return a command

  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Climber encoder", m_encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
