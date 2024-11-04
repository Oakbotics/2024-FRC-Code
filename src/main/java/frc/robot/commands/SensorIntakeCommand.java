// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class SensorIntakeCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SensorIntakeCommand(ConveyorSubsystem conveyorSubsystem, ShooterSubsystem shooterSubsystem, boolean useSensor) {
    m_conveyorSubsystem = conveyorSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_conveyorSubsystem, m_shooterSubsystem);

    addCommands(
        new IntakeCommand(m_conveyorSubsystem, useSensor),
        new OuttakeShooterConveyorCommand(m_conveyorSubsystem, m_shooterSubsystem, useSensor)
        /*
        new InstantCommand(()-> m_conveyorSubsystem.resetOdometry(AutoConstants.middleStartingPose)),
        new RevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem),
        new ParallelCommandGroup(
          new IntakeCommand(m_conveyorSubsystem, true),
          new GoToAutoPositionCommand(m_conveyorSubsystem, AutoConstants.bC2Pose)
        ),
        // new GoToAutoPositionCommand(m_driveSubsystem, AutoConstants.middleFarShootPose),
          // use this line if GoToSpeakerCommand doesnt work
        new PnuematicsReverseCommand(m_pnuematicSubsystem),
        new GoToSpeakerCommand(m_conveyorSubsystem),
        new RevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem)
        */
    );
  }
}