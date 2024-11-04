// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An intake command that uses an intake subsystem. */
public class IndexCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean useSensor = true;

  /**
   * Creates a new IndexCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexCommand(ConveyorSubsystem subsystem, boolean useSensor, ShooterSubsystem shooterSubsystem) {
    m_conveyorSubsystem = subsystem;
    m_shooterSubsystem = shooterSubsystem;
    this.useSensor = useSensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_conveyorSubsystem.runConveyorSpeed(0.25);
    m_shooterSubsystem.runShooterSpeed(-2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyorSubsystem.runConveyorSpeed(0);
    m_shooterSubsystem.runShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if(!useSensor) return false;

    return m_conveyorSubsystem.getBottomSensorTriggered();
  }
}