// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RetractClimberCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_climberSubsystem;

  /**
   * Creates a new RetractCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RetractClimberCommand(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_climberSubsystem.setClimberSpeed(-0.5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_climberSubsystem.getEncoderValue() < 5){
      return true;
    }
     return false;
    
  }
}

