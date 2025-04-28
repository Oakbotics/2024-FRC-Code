package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteLimelightSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class GoToNoteCommandGroup extends ParallelCommandGroup {

    
  public GoToNoteCommandGroup(ConveyorSubsystem m_conveyorSubsystem, DriveSubsystem m_driveSubsystem, ShooterSubsystem m_shooterSubsystem, NoteLimelightSubsystem m_limelightSubsystem, PnuematicSubsystem m_pnuematicSubsystem){

    addCommands(
      new GoToNoteCommand(m_driveSubsystem, m_limelightSubsystem),
      new SensorIntakeCommand(m_conveyorSubsystem, m_shooterSubsystem, true),
      new PnuematicsForwardCommand(m_pnuematicSubsystem)

    );

    addRequirements(m_driveSubsystem, m_limelightSubsystem, m_shooterSubsystem, m_pnuematicSubsystem, m_conveyorSubsystem);
  }

}