// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANSparkMax m_topShooterMotor;
  private final CANSparkMax m_bottomShooterMotor;

  public ShooterSubsystem() {

    m_topShooterMotor = new CANSparkMax(ConveyorConstants.kRightShooterMotorCANID, MotorType.kBrushless);
    m_bottomShooterMotor = new CANSparkMax(ConveyorConstants.kLeftShooterMotorCANID, MotorType.kBrushless);

    m_topShooterMotor.setSmartCurrentLimit(30);
    m_bottomShooterMotor.setSmartCurrentLimit(30);

    m_bottomShooterMotor.restoreFactoryDefaults();
    m_topShooterMotor.restoreFactoryDefaults();    

    // m_bottomShooterMotor.setInverted(false);
    // m_topShooterMotor.setInverted(false);

    m_bottomShooterMotor.setIdleMode(IdleMode.kBrake);
    m_topShooterMotor.setIdleMode(IdleMode.kBrake);    
  }

  public void runShooterSpeed(double speed){
    m_topShooterMotor.setVoltage(-speed);
    m_bottomShooterMotor.setVoltage(-speed);
  }

  public double getShooterVelocity(){

    return (m_topShooterMotor.getEncoder().getVelocity() + m_bottomShooterMotor.getEncoder().getVelocity())/2;

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
