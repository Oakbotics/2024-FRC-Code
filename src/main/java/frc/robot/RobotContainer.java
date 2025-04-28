// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToNoteCommandGroup;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.KACalculatorCommand;
import frc.robot.commands.OuttakeShooterConveyorCommand;
import frc.robot.commands.PnuematicsForwardCommand;
import frc.robot.commands.PnuematicsReverseCommand;
import frc.robot.commands.ResetGyroUsingAprilTag;
import frc.robot.commands.SensorBottomIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootCommandGroup;
import frc.robot.commands.VelocityTuningCommand;
import frc.robot.commands.autoCommands.autoCommandGroups.M_4P;
import frc.robot.commands.autoCommands.autoCommandGroups.M_4PFF;
import frc.robot.commands.autoCommands.autoCommandGroups.M_4PNoteAlignBlueAmp;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NoteLimelightSubsystem;
import frc.robot.subsystems.AprilTagLimelightSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final CandleSubsystem m_candleSubsystem = new CandleSubsystem();

  // private final SendableChooser<String> autoChooser;
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
    
    // NamedCommands.registerCommand("ResetGyro0", new InstantCommand(()-> m_driveSubsystem.setGyro(0)));
    // NamedCommands.registerCommand("ResetGyro180", new InstantCommand(()-> m_driveSubsystem.setGyro(180)));
    // NamedCommands.registerCommand("shoot", new RevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem).withTimeout(1.5));
    // NamedCommands.registerCommand("Intake", new IntakeCommand(m_conveyorSubsystem, true).withTimeout(2));
    // NamedCommands.registerCommand("Rev Shooter", new ShootCommand(m_shooterSubsystem).withTimeout(15));
    // NamedCommands.registerCommand("Intake Into Shooter", new IntakeShootCommand(m_conveyorSubsystem, false));
    // NamedCommands.registerCommand("Pneumatics Up", new PnuematicsForwardCommand(m_pnuematicSubsystem));
    // NamedCommands.registerCommand("Pneumatics Down", new PnuematicsReverseCommand(m_pnuematicSubsystem));



    // autoChooser = new SendableChooser<>();
    // autoChooser.setDefaultOption("JustCloseMiddle", "JustCloseMiddle");
    // autoChooser.addOption("MoveStraight180", "MoveStraight180");
    // autoChooser.addOption("MoveStraight", "MoveStraight");
    // autoChooser.addOption("Two piece", "Two piece");
    // SmartDashboard.putData("Auto Chooser", autoChooser);
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
// <<<<<<< DCMP
    // m_operatorController.leftBumper().whileTrue(new SensorIntakeCommand(m_conveyorSubsystem, true));
    //m_operatorController.rightTrigger().whileTrue(new RevThenShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem));
    // m_operatorController.a().whileTrue(new RetractClimberCommand(m_climberSubsystem));
    // m_operatorController.y().whileTrue(new ExtendClimberCommand(m_climberSubsystem));
    m_operatorController.povUp().whileTrue(new OuttakeShooterConveyorCommand(m_conveyorSubsystem, m_shooterSubsystem, false));
    m_operatorController.leftTrigger().whileTrue(new ShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem));
    m_operatorController.x().whileTrue(new PnuematicsReverseCommand(m_pnuematicSubsystem)).onFalse(new PnuematicsForwardCommand(m_pnuematicSubsystem));
    // m_operatorController.b().whileTrue(new PnuematicsForwardCommand(m_pnuematicSubsystem));    
    // operator controls for shoot while rev - shoot command and rev command
    // switched rev up to left bumper, and shoot piece to right trigger
    m_operatorController.rightTrigger().whileTrue(new ShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem));
    m_operatorController.leftBumper().whileTrue(new ShootCommand(m_shooterSubsystem));



    // m_driverController.a().whileTrue(new GoToAmpCommand(m_driveSubsystem));
    // m_driverController.leftBumper().whileTrue(new GoToSpeakerCommand(m_driveSubsystem));
// =======
  
//     // m_driverController.b().whileTrue(new OuttakeShooterConveyorCommand(m_conveyorSubsystem, m_shooterSubsystem, false));
//     //m_driverController.y().onTrue(new PnuematicsForwardCommand(m_pnuematicSubsystem));
//     m_driverController.y().onTrue(
//       new InstantCommand(() -> {
//         m_driveSubsystem.resetBotPose(new Pose2d(1.42, 1.6, new Rotation2d(0.0)));
//       })
//     );
//     // m_driverController.a().onTrue(new PnuematicsReverseCommand(m_pnuematicSubsystem));
//     m_driverController.a().whileTrue(new KACalculatorCommand(m_driveSubsystem));
//     m_driverController.b().whileTrue(new VelocityTuningCommand(m_driveSubsystem));
//     // m_driverController.x().whileTrue(new GoToNoteCommandGroup(m_conveyorSubsystem, m_driveSubsystem, m_shooterSubsystem, m_noteLimelightSubsystem, m_pnuematicSubsystem)); 
//     m_driverController.x().onTrue(
//       new InstantCommand(() -> {
//         m_driveSubsystem.resetOdometry(new Pose2d(1.42, 1.6, new Rotation2d(0.0)));
//       })
//     );
//     m_driverController.rightTrigger().whileTrue(new ShootCommandGroup(m_conveyorSubsystem, m_shooterSubsystem));
//     m_driverController.rightBumper().whileTrue(new ShootCommand(m_shooterSubsystem));
// >>>>>>> main
    m_driverController.leftBumper().whileTrue((new SensorBottomIntakeCommand(m_conveyorSubsystem, true, m_shooterSubsystem))).onFalse(new IndexCommand(m_conveyorSubsystem, true, m_shooterSubsystem));
    m_driverController.povUp().onTrue(new ResetGyroUsingAprilTag(m_driveSubsystem));
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_driveSubsystem.setGyro(0)));

    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right st
        new RunCommand(
            () -> m_driveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.5 - m_driverController.getLeftTriggerAxis()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * (1.5 - m_driverController.getLeftTriggerAxis()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * (1.5 - m_driverController.getLeftTriggerAxis()), OIConstants.kDriveDeadband),
                true, true),
            m_driveSubsystem));

    new Trigger(
        () -> m_conveyorSubsystem.getNoteAligned() == true

      ).onTrue(
        new InstantCommand(()-> m_candleSubsystem.setGreen())
        .andThen(
          new RunCommand(() -> {
              m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
          })
        )
        .withTimeout(2)
        .andThen(
          new InstantCommand(() -> {
            m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
            m_candleSubsystem.setRainbowAnimation();
          })
        )
    );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *    
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new PathPlannerAuto("Diagonal");''
    // return new M_4PFF(m_driveSubsystem, m_shooterSubsystem, m_conveyorSubsystem, m_pnuematicSubsystem);
    // return new M_4PNoteAlignBlueAmp(m_driveSubsystem, m_shooterSubsystem, m_conveyorSubsystem, m_pnuematicSubsystem, m_noteLimelightSubsystem);
    // SmartDashboard.putString("WPILIB DEPLOYE DIRECTORY", Filesystem.getDeployDirectory().getPath());
    return new PathPlannerAuto("MoveStraight");
  }
}
