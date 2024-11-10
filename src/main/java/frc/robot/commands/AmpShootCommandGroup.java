package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShootCommandGroup extends SequentialCommandGroup{
  public AmpShootCommandGroup(ConveyorSubsystem m_conveyorSubsystem, ShooterSubsystem m_shooterSubsystem){
    addCommands(
    new AmpShootCommand(m_shooterSubsystem, m_conveyorSubsystem).withTimeout(2),
    new AmpShootCommand(m_shooterSubsystem, m_conveyorSubsystem).alongWith(new ConveyorCommand(m_conveyorSubsystem))
    );

    addRequirements(
      m_conveyorSubsystem,
      m_shooterSubsystem);
  }
}

  
