// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VelocityTuningCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final ExampleSubsystem m_subsystem;

  double maxVel;
  double target;
  SlewRateLimiter limiter;
  Timer timer;
  double previousVel;

  DriveSubsystem m_driveSubsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VelocityTuningCommand(DriveSubsystem subsystem) {
    m_driveSubsystem = subsystem;

    SmartDashboard.putNumber("Sysid VelTarget", 0);
    SmartDashboard.putNumber("Sysid Vel", 0);
    SmartDashboard.putNumber("Sysid Accel", 0);
    SmartDashboard.putNumber("Sysid BatteryVoltage", 0); 
    SmartDashboard.putNumber("KsVoltage", 1);
    // m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    maxVel = 2 / Constants.DriveConstants.kTeleopMaxSpeedMetersPerSecond; // desired max / max speed
    target = maxVel;
    limiter = new SlewRateLimiter(5);
    // timer = new Timer();
    previousVel = 0;

    SmartDashboard.putNumber("Sysid VelTarget", 0);
    SmartDashboard.putNumber("Sysid Vel", 0);
    SmartDashboard.putNumber("Sysid Accel", 0);
    SmartDashboard.putNumber("Sysid BatteryVoltage", 0);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vel = limiter.calculate(target);

    // if(vel == maxVel){
    //     timer.start();
    // }

    // if(timer.get() > 0.5){
    //     timer.reset();
    //     target = 0;
    // }

    m_driveSubsystem.drive(vel, 0, 0, false, false);
    SmartDashboard.putNumber("Sysid VelTarget", vel);
    SmartDashboard.putNumber("Sysid Vel", m_driveSubsystem.getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Sysid Encoder Vel", m_driveSubsystem.getEncoderVelocity());
    SmartDashboard.putNumber("Sysid Accel", (vel - previousVel)/50);
    SmartDashboard.putNumber("Sysid BatteryVoltage", RobotController.getBatteryVoltage());  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // timer.reset();
    target = 0;
    SmartDashboard.putNumber("Sysid VelTarget", 0);
    SmartDashboard.putNumber("Sysid Vel", 0);
    SmartDashboard.putNumber("Sysid Accel", 0);
    SmartDashboard.putNumber("Sysid BatteryVoltage", 0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
