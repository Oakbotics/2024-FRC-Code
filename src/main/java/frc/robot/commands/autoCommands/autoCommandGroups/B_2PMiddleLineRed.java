// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoCommandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.GoToNoteCommandGroup;
import frc.robot.commands.GoToPositionCommand;
import frc.robot.commands.GoToSpeakerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PnuematicsForwardCommand;
import frc.robot.commands.PnuematicsReverseCommand;
import frc.robot.commands.RevThenShootCommandGroup;
import frc.robot.commands.SensorIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootCommandGroup;
import frc.robot.commands.autoCommands.GoToAutoPositionCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteLimelightSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class B_2PMiddleLineRed extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final PnuematicSubsystem m_pnuematicSubsystem;
  private final NoteLimelightSubsystem m_noteLimelightSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public B_2PMiddleLineRed(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, PnuematicSubsystem pnuematicSubsystem, NoteLimelightSubsystem noteLimelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_conveyorSubsystem = conveyorSubsystem;
    m_pnuematicSubsystem = pnuematicSubsystem;
    m_noteLimelightSubsystem = noteLimelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);

    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(()-> m_driveSubsystem.resetBotPose(AutoConstants.redBottomStartingPose)),
        new PnuematicsForwardCommand(m_pnuematicSubsystem),
        new RevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem),

        new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.redM4GetOutOfWayPose).withTimeout(1.5),
        new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.redM4LineupPose).withTimeout(1.5),

        new GoToNoteCommandGroup(conveyorSubsystem, driveSubsystem, shooterSubsystem, noteLimelightSubsystem, pnuematicSubsystem).withTimeout(2),

        new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.redM4GetOutOfWayPose).withTimeout(1.5),

        new ParallelCommandGroup(
          new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.redBottomStartingPose).withTimeout(2),
          new ShootCommand(m_shooterSubsystem).withTimeout(2)
        ),

        new InstantCommand( ()-> m_driveSubsystem.setX()),
        new ShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem)

      ) 
    );
  }
}