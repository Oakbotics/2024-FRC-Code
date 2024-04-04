
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An intake command that uses an intake subsystem. */
public class OuttakeShooterConveyorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean useSensor = true;

  /**
   * Creates a new intakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OuttakeShooterConveyorCommand (ConveyorSubsystem subsystem, ShooterSubsystem shooterSubsystem, boolean useSensor) {
    m_conveyorSubsystem = subsystem;
    m_shooterSubsystem = shooterSubsystem;
    this.useSensor = useSensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyorSubsystem.runConveyorSpeed(0.15);
    m_shooterSubsystem.runShooterSpeed(-12);
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
    if(!useSensor) return false;
    // else return m_conveyorSubsystem.getSensorTriggered();
    return m_conveyorSubsystem.getNoteAligned();
    // return false;
  }
}
