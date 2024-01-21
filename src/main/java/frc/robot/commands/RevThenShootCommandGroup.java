package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class RevThenShootCommandGroup extends SequentialCommandGroup {

    
  public RevThenShootCommandGroup(ConveyorSubsystem subsystem){

    addCommands(
     new ShootCommand(subsystem).withTimeout(2),
     new ShootCommand(subsystem).alongWith(new ConveyorCommand(subsystem))


    );
  }

}