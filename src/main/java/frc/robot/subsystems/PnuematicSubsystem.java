// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Constants;

public class PnuematicSubsystem extends SubsystemBase {
  /** Creates a new PnuematicSubsystem. */
  private final Solenoid leftSolenoid;
  private final Solenoid rightSolenoid;

  public PnuematicSubsystem() {
    leftSolenoid = new Solenoid(PnuematicsModuleType.REVPH, PnuematicConstants.kLeftSolenoid);
    rightSolenoid = new Solenoid(PnuematicsModuleType.REVPH, PnuematicConstants.kRightSolenoid);
  }

  public void setState(boolean state){
    leftSolenoid.set(state);
    rightSolenoid.set(state);
  }

  public void toggleState()
  {
    leftSolenoid.toggle();
    rightSolenoid.toggle();
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
 