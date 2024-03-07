// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.GoToPositionCommand;
import frc.robot.commands.GoToSpeakerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PnuematicsReverseCommand;
import frc.robot.commands.RevThenShootCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autoCommands.GoToAutoPositionCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class SensorIntakeCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ConveyorSubsystem m_conveyorSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SensorIntakeCommand(ConveyorSubsystem conveyorSubsystem, boolean useSensor) {
    m_conveyorSubsystem = conveyorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_conveyorSubsystem);

    addCommands(
        new IntakeCommand(m_conveyorSubsystem, useSensor),
        new OuttakeCommand(m_conveyorSubsystem, useSensor)

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