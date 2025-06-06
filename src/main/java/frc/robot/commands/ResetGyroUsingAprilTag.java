// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilTagLimelightSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An intake command that uses an intake subsystem. */
public class ResetGyroUsingAprilTag extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final AprilTagLimelightSubsystem m_limelightSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  /**
   * Creates a new intakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ResetGyroUsingAprilTag( DriveSubsystem drivesubsytem) {
    // m_limelightSubsystem = limelightsubsystem;
    m_driveSubsystem = drivesubsytem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivesubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        m_driveSubsystem.setGyroYawUsingAprilTag();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_conveyorSubsystem.getSensorTriggered();
    return true;

  }
}
