// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.PnuematicSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TogglePnuematicsCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PnuematicSubsystem m_pnuematicSubsystem;

  /**
   * Creates a new TogglePnuematicsCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TogglePnuematicsCommand(PnuematicSubsystem pnuematicSubsystem) {
    m_pnuematicSubsystem = pnuematicSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pnuematicSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pnuematicSubsystem.toggleState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  
  public boolean isFinished() {
    return false;
  }
}
