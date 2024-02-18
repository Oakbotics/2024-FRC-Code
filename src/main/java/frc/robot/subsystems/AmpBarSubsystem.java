// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpBarConstants;

public class AmpBarSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public CANSparkMax m_motorLeft;
  public CANSparkMax m_motorRight;
  private SparkPIDController m_pidControllerLeft;
  private SparkPIDController m_pidControllerRight;
  private AbsoluteEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private final double encoderMultiplier = (Units.radiansToDegrees(Math.PI * 2));   //Degrees

  private  float MAXPosition = 45;
  private float MINPosition = 5;


  public AmpBarSubsystem() {

    // initialize motor
    m_motorLeft = new CANSparkMax(AmpBarConstants.kAmpBarLeftCANID, MotorType.kBrushless);
    m_motorRight = new CANSparkMax(AmpBarConstants.kAmpBarRightCANID, MotorType.kBrushless);


    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motorLeft.restoreFactoryDefaults();
    m_motorRight.restoreFactoryDefaults();


    // initialze PID controller and encoder objects
    m_pidControllerLeft = m_motorLeft.getPIDController();
    m_pidControllerRight = m_motorRight.getPIDController();

    m_encoder = m_motorLeft.getAbsoluteEncoder(Type.kDutyCycle);

    m_pidControllerLeft.setFeedbackDevice(m_encoder);
    m_pidControllerRight.setFeedbackDevice(m_encoder);
    m_encoder.setPositionConversionFactor(encoderMultiplier);
    
    m_encoder.setInverted(true);
    m_motorLeft.setInverted(false);

    // PID coefficients
    // kP = 0.006;
    // kI = 0;
    // kD = 0.003; 
    // kIz = 0; 
    // kFF = 0.003; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;

    kP = 0.04; 
    kI = 0;
    kD = 0.003; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.5; 
    kMinOutput = -0.5;
    //maxRPM = 5700;
    //allowedErr = 0.1;
  

    // Smart Motion Coefficients

    maxVel = 7; // rpm
    maxAcc = 1;

    // set PID coefficients
    m_pidControllerLeft.setP(kP);
    m_pidControllerLeft.setI(kI);
    m_pidControllerLeft.setD(kD);
    m_pidControllerLeft.setIZone(kIz);
    m_pidControllerLeft.setFF(kFF);
    m_pidControllerLeft.setOutputRange(kMinOutput, kMaxOutput);

    m_pidControllerRight.setP(kP);
    m_pidControllerRight.setI(kI);
    m_pidControllerRight.setD(kD);
    m_pidControllerRight.setIZone(kIz);
    m_pidControllerRight.setFF(kFF);
    m_pidControllerRight.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */

    // int smartMotionSlot = 0;
    // m_pidControllerLeft.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    // m_pidControllerLeft.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    // m_pidControllerLeft.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    // m_pidControllerLeft.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    // m_pidControllerLeft.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, smartMotionSlot);

    m_motorLeft.setIdleMode(IdleMode.kCoast);
    m_motorRight.setIdleMode(IdleMode.kCoast);

    m_motorLeft.setSoftLimit(SoftLimitDirection.kForward, MAXPosition);
    m_motorLeft.setSoftLimit(SoftLimitDirection.kReverse, MINPosition);
    m_motorRight.setSoftLimit(SoftLimitDirection.kForward, MAXPosition);
    m_motorRight.setSoftLimit(SoftLimitDirection.kReverse, MINPosition);

    m_motorLeft.setClosedLoopRampRate(0.2);
    m_motorRight.setClosedLoopRampRate(0.2);

    m_motorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_motorLeft.setSmartCurrentLimit(40);
    m_motorRight.setSmartCurrentLimit(40);


    
  }

  public void MoveAmpBarDegrees (double degrees) {

      // if(m_encoder.getPosition() >= MAXPosition) m_pidControllerLeft.setReference(MAXPosition, CANSparkMax.ControlType.kPosition); 
      // else if(m_encoder.getPosition() <= MINPosition) m_pidControllerLeft.setReference(MINPosition, CANSparkMax.ControlType.kPosition);  
      m_pidControllerLeft.setReference(degrees, CANSparkMax.ControlType.kPosition);
      m_pidControllerRight.setReference(degrees, CANSparkMax.ControlType.kPosition); 

  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
