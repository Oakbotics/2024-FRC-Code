// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoCommandGroups;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.PnuematicsForwardCommand;
import frc.robot.commands.AutoRevThenShootCommandGroup;
import frc.robot.commands.ShootCommandGroup;
import frc.robot.commands.autoCommands.GoToAutoPositionCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteLimelightSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class S_2PBlue extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final PnuematicSubsystem m_pnuematicSubsystem;
  // private final NoteLimelightSubsystem m_noteLimelightSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public S_2PBlue(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, PnuematicSubsystem pnuematicSubsystem, NoteLimelightSubsystem noteLimelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_conveyorSubsystem = conveyorSubsystem;
    m_pnuematicSubsystem = pnuematicSubsystem;
    // m_noteLimelightSubsystem = noteLimelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);

    addCommands(
      new SequentialCommandGroup(      
        new RunCommand(()-> m_driveSubsystem.resetBotPose(AutoConstants.bottomStartingPose)).withTimeout(0.2),
        new PnuematicsForwardCommand(m_pnuematicSubsystem),
        new AutoRevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem),
        
        new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.blueM4GetOutOfWayPose).withTimeout(3),
        // new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.blueM4LineupPose).withTimeout(2),

        // new GoToNoteCommandGroup(conveyorSubsystem, driveSubsystem, shooterSubsystem, m_noteLimelightSubsystem, pnuematicSubsystem).withTimeout(2),

        // new GoToAutoPositionCommand(driveSubsystem, ()-> AutoConstants.blueM4GetOutOfWayPose).withTimeout(2),
        // new ParallelCommandGroup(
        //   new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.bottomStartingPose).withTimeout(3),
        //   new ShootCommand(shooterSubsystem).withTimeout(2.5)
        // ),
        // new InstantCommand(() -> m_driveSubsystem.setX()),
        new ShootCommandGroup(conveyorSubsystem, shooterSubsystem)



      )
    ); 
  }
}