// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PnuematicSubsystem extends SubsystemBase {
  /** Creates a new PnuematicSubsystem. */
  private final DoubleSolenoid m_doubleSolenoid;

  public PnuematicSubsystem() {
    m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void setStateForward(){
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setStateReverse(){
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }


  public void toggleState()
  {
    m_doubleSolenoid.toggle();
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
 