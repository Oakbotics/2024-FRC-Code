package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class AutoRevThenShootCommandGroup extends SequentialCommandGroup {

    
  public AutoRevThenShootCommandGroup(ConveyorSubsystem m_conveyorSubsystem, ShooterSubsystem m_shooterSubsystem){

    addCommands(
     new AutoShootCommand(m_shooterSubsystem).withTimeout(1.5),
     new AutoShootCommand(m_shooterSubsystem).alongWith(new ConveyorCommand(m_conveyorSubsystem)).withTimeout(0.5)

    );

    addRequirements(m_shooterSubsystem, m_conveyorSubsystem);
  }

}