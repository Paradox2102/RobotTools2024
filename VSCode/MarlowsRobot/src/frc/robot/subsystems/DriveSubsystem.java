/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Encoder;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class DriveSubsystem extends SubsystemBase {
  private static final int k_leftMotorPWMPin = Device.M1_2_PWM;
  private static final int k_leftMotorDirPin = Device.M1_2_DIR;
  private static final int k_rightMotorPWMPin = Device.M1_1_PWM;
  private static final int k_rightMotorDirPin = Device.M1_1_DIR;

  private static final int k_leftEncoderPin1 = Device.Q1_INT;
  private static final int k_leftEncoderPin2 = Device.Q1_DIR;
  private static final int k_rightEncoderPin1 = Device.Q2_INT;
  private static final int k_rightEncoderPin2 = Device.Q2_DIR;
  
  private Encoder m_leftEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_leftEncoderPin1, k_leftEncoderPin2);
  private Encoder m_rightEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_rightEncoderPin1, k_rightEncoderPin2);

  private PWMMotor m_leftmotor = new PWMMotor(k_leftMotorPWMPin, k_leftMotorDirPin);
  private PWMMotor m_rightmotor = new PWMMotor(k_rightMotorPWMPin, k_rightMotorDirPin);

  public static final double k_maxSpeed = 1075; 
  private static final double k_minPowerLeft = 0.3;
  private static final double k_minPowerRight = 0.3;
  private static final double k_Fleft = 0.95/k_maxSpeed;
  private static final double k_Fright = 0.95/k_maxSpeed;
  /* 
  *private static final double k_P = 0.0002;
  *private static final double k_I = 0.001;
  *private static final double k_IZone = 50;
   */

  
  /**
   * 
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");

    m_leftmotor.setFeedbackDevice(m_leftEncoder);
    m_rightmotor.setFeedbackDevice(m_rightEncoder);

  
    m_leftmotor.setMaxSpeed(k_maxSpeed);
    m_rightmotor.setMaxSpeed(k_maxSpeed);
    

    m_rightEncoder.setInverted(true);

    m_leftmotor.setMinPower(k_minPowerLeft);
    m_rightmotor.setMinPower(k_minPowerRight);

    m_leftmotor.setFTerm(k_Fleft);
    m_rightmotor.setFTerm(k_Fright);

    /* 
    m_leftmotor.setPTerm(k_P);
    m_rightmotor.setPTerm(k_P);

    m_leftmotor.setITerm(k_I);
    m_rightmotor.setITerm(k_I);

    m_leftmotor.setIZone(k_IZone);
    m_rightmotor.setIZone(k_IZone);
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");

  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    m_leftmotor.setControlMode(SmartMotorMode.Speed);
    m_rightmotor.setControlMode(SmartMotorMode.Speed);
    m_leftmotor.set(leftSpeed);
    m_rightmotor.set(rightSpeed);

  }
  public void setPower(double leftpower, double rightpower){
    m_leftmotor.setControlMode(SmartMotorMode.Power);
    m_rightmotor.setControlMode(SmartMotorMode.Power);
    
    m_leftmotor.set(leftpower);
    m_rightmotor.set(rightpower);

  }  

  public Encoder getLeftEncoder() {
    return (m_leftEncoder);
  }

  public Encoder getRightEncoder() {
    return (m_rightEncoder);
  }  
}
