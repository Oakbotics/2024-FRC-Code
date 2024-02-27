// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToAmpCommand;
import frc.robot.commands.GoToNoteCommand;
import frc.robot.commands.GoToSpeakerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetGyroUsingAprilTag;
import frc.robot.commands.RevThenShootCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TogglePnuematicsCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NoteLimelightSubsystem;
import frc.robot.subsystems.AprilTagLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final AprilTagLimelightSubsystem m_aprilTagLimelightSubsystem = new AprilTagLimelightSubsystem();
  private final NoteLimelightSubsystem m_noteLimelightSubsystem = new NoteLimelightSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_aprilTagLimelightSubsystem);

  private final SendableChooser<String> autoChooser;
  private final PnuematicSubsystem m_pnuematicSubsystem = new PnuematicSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    NamedCommands.registerCommand("ResetGyro0", new InstantCommand(()-> m_driveSubsystem.setGyro(0)));
    NamedCommands.registerCommand("ResetGyro180", new InstantCommand(()-> m_driveSubsystem.setGyro(180)));
    NamedCommands.registerCommand("shoot", new RevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem).withTimeout(1.5));
    NamedCommands.registerCommand("intake", new IntakeCommand(m_conveyorSubsystem).withTimeout(2));

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("JustCloseMiddle", "JustCloseMiddle");
    autoChooser.addOption("MoveStraight180", "MoveStraight180");
    autoChooser.addOption("MoveStraight", "MoveStraight");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   *  
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_operatorController.leftBumper().whileTrue(new IntakeCommand(m_conveyorSubsystem));
    m_operatorController.rightBumper().whileTrue(new RevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem));
    m_operatorController.a().whileTrue(new TogglePnuematicsCommand(m_pnuematicSubsystem));

    m_driverController.b().whileTrue(new GoToNoteCommand(m_driveSubsystem, m_noteLimelightSubsystem));
    m_driverController.a().whileTrue(new GoToAmpCommand(m_driveSubsystem));
    m_driverController.y().whileTrue(new GoToSpeakerCommand(m_driveSubsystem));
    m_driverController.povUp().onTrue(new ResetGyroUsingAprilTag(m_aprilTagLimelightSubsystem, m_driveSubsystem));
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_driveSubsystem.setGyro(0)));

  
    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right st
        new RunCommand(
            () -> m_driveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.5 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * (1.5 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * (1.5 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband),
                true, true),
            m_driveSubsystem));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *    
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto(autoChooser.getSelected());
  }
}
