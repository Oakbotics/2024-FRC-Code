// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoCommandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.PnuematicsForwardCommand;
import frc.robot.commands.AutoRevThenShootCommandGroup;
import frc.robot.commands.SensorIntakeCommand;
import frc.robot.commands.autoCommands.GoToAutoPositionCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/** An example command that uses an example subsystem. */
public class B_2P extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final PnuematicSubsystem m_pnuematicSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public B_2P(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, PnuematicSubsystem pnuematicSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_conveyorSubsystem = conveyorSubsystem;
    m_pnuematicSubsystem = pnuematicSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);

    addCommands(
      new SequentialCommandGroup(
        
        new InstantCommand(()-> m_driveSubsystem.resetBotPose(AutoConstants.bottomStartingPose)),
        new PnuematicsForwardCommand(m_pnuematicSubsystem),
        new AutoRevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem),
        new ParallelCommandGroup(
          new SensorIntakeCommand(m_conveyorSubsystem, m_shooterSubsystem, true).withTimeout(5),
          new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.bC3Pose).withTimeout(5)
        ),
        // new GoToAutoPositionCommand(m_driveSubsystem, ()-> new Pose2d(2.6, 4.5, Rotation2d.fromDegrees(0))),
        // new PnuematicsReverseCommand(m_pnuematicSubsystem).withTimeout(0.2),
        new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.bottomStartingPose).withTimeout((2)),
        new InstantCommand(() -> m_driveSubsystem.setX()),
        new AutoRevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem)
      )
    );
  }
}