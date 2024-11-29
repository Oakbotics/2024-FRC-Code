// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoCommandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.GoToNoteCommandGroup;
import frc.robot.commands.PnuematicsForwardCommand;
import frc.robot.commands.AutoRevThenShootCommandGroup;
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
public class M_4PNoteAlignBlueAmp extends SequentialCommandGroup {
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
  public M_4PNoteAlignBlueAmp(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, PnuematicSubsystem pnuematicSubsystem, NoteLimelightSubsystem noteLimelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_conveyorSubsystem = conveyorSubsystem;
    m_pnuematicSubsystem = pnuematicSubsystem;
    m_noteLimelightSubsystem = noteLimelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);

    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(()-> m_driveSubsystem.resetOdometry(new Pose2d(AutoConstants.middleStartingPose.getX() + 0.02, AutoConstants.middleStartingPose.getY(), Rotation2d.fromDegrees(0)))),
        new PnuematicsForwardCommand(m_pnuematicSubsystem),
        
        new RunCommand(()->{}).withTimeout(2),        
        new AutoRevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem),


        new ParallelCommandGroup(
          new SensorIntakeCommand(m_conveyorSubsystem, m_shooterSubsystem, true).withTimeout(2.5),
          new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.bC2Pose).withTimeout(2.5)
        ),
        // new GoToAutoPositioCommand(m_driveSubsystem, AutoConstants.middleFarShootPose),
          // use this line ^^ if GoToSpeakerCommand doesnt workn
        new ParallelCommandGroup(
          new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.middleStartingPose).withTimeout(1.5),
           new ShootCommand(m_shooterSubsystem).withTimeout(1.5)
        ),
        new ShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem),

        new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.blueAmpNoteLineupPose).withTimeout(2),

        new GoToNoteCommandGroup(m_conveyorSubsystem, m_driveSubsystem, m_shooterSubsystem, m_noteLimelightSubsystem, m_pnuematicSubsystem).withTimeout(2),

        new ParallelCommandGroup(
          new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.subwooferShootSafe).withTimeout(1.5),
          new ShootCommand(m_shooterSubsystem).withTimeout(1.5)
        ),
        new InstantCommand(() -> m_driveSubsystem.setX()),
        new ShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem),

        new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.bluePodiumNoteLineupPose).withTimeout(2)
          .alongWith(new SensorIntakeCommand(m_conveyorSubsystem, m_shooterSubsystem, false).withTimeout(1)),
        
        new GoToNoteCommandGroup(m_conveyorSubsystem, m_driveSubsystem, m_shooterSubsystem, m_noteLimelightSubsystem, m_pnuematicSubsystem).withTimeout(2),

        new ParallelCommandGroup(
          new GoToAutoPositionCommand(m_driveSubsystem, ()-> AutoConstants.subwooferShootSafe).withTimeout(1.5),
          new ShootCommand(m_shooterSubsystem).withTimeout(1.5)
        ),

        new InstantCommand(() -> m_driveSubsystem.setX()),
        new ShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem)              
      ) 
    );
  }
}