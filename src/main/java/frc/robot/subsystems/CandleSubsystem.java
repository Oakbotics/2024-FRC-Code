// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANdleConstants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;



public class CandleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */


  private final CANdle m_candle;
  private final int ledCount = 28;

  public CandleSubsystem() {

    m_candle = new CANdle(CANdleConstants.CANdleCanID);

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.5;
    configAll.vBatOutputMode = VBatOutputMode.On;   
    
    
    m_candle.configAllSettings(configAll, 100);
    setRainbowAnimation();

  }


  public void setRainbowAnimation(){
    m_candle.animate(new RainbowAnimation(1, 0.3, ledCount, true, 0));
  }

  public void setRed(){
    m_candle.setLEDs(255, 0, 0, 0, 0, ledCount);
  }
  public void setGreen(){
    m_candle.animate(new ColorFlowAnimation(185, 59, 93, 0, 0.7, ledCount, Direction.Forward));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
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
