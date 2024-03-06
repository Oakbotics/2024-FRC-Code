// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RevThenShootCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class T_2P extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ConveyorSubsystem m_conveyorSubsystem;
  // private final CandleSubsystem m_candleSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public  T_2P(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_conveyorSubsystem = conveyorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);

    addCommands(

        new RevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem)
      
        // new InstantCommand(()-> m_driveSubsystem.zeroHeading(), m_driveSubsystem),
        // new ArmCommandLow(m_armSubsystem),
        // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-0.3, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
        // new ArmCommandHigh(armSubsystem),
        // new GoDistanceSwerveCommand(m_driveSubsystem, m_limelightSubsystem,  new Pose2d(new Translation2d(-0.3,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
        // new OuttakeCommand(m_intakeSubsystem).repeatedly().withTimeout(1),
        // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-0.3, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
        // new ArmCommandLow(m_armSubsystem),
        // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(-0.3,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-4, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
        // new GoDistanceSwerveCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(-4,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-2.4, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
        // new InstantCommand(()-> m_driveSubsystem.zeroHeading(180), m_driveSubsystem),
        // new RunCommand(()-> m_driveSubsystem.setX(), m_driveSubsystem)
       
        );
    }
}