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

  
  
  public static final double k_maxSpeed = 1600;
  public static final double k_minPowerLeft = .3;
  public static final double k_minPowerRight = .3;

  public static final double k_Fleft = 0.96/k_maxSpeed;
  public static final double k_Fright = 0.95/k_maxSpeed;

  private static final double k_P = 0.003;

  private static final double k_I = 0.000;
  private static final double k_IZone = 50;

   //Define motor constructor arguments before Constructor calls
 private static final int k_leftMotorPWMPin = Device.M1_2_PWM;
  private static final int k_leftMotorDirPin = Device.M1_2_DIR;
  private static final int k_rightMotorPWMPin = Device.M1_1_PWM;
  private static final int k_rightMotorDirPin = Device.M1_1_DIR;

   //Define encoder constructor arguments before Constructor calls
   private static final int k_leftEncoderPin1 = Device.Q1_INT;
   private static final int k_leftEncoderPin2 = Device.Q1_DIR;
   private static final int k_rightEncoderPin1 = Device.Q2_INT;
   private static final int k_rightEncoderPin2 = Device.Q2_DIR;

// Define motor variables
  //and Instantiate and assign PWMMotor objects, left and right
  private PWMMotor m_leftMotor = new PWMMotor(k_leftMotorPWMPin, k_leftMotorDirPin);
  private PWMMotor m_righMotor = new PWMMotor(k_rightMotorPWMPin, k_rightMotorDirPin);

   //Instantiate and assign Encoder objtecs, left and right.
   private Encoder m_leftEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_leftEncoderPin1, k_leftEncoderPin2);
   private Encoder m_rightEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_rightEncoderPin1, k_rightEncoderPin2);

   

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");

    m_rightEncoder.setInverted(true);

    m_leftMotor.setMinPower(k_minPowerLeft);
    m_righMotor.setMinPower(k_minPowerRight);
    m_leftMotor.setFTerm(k_Fleft);
    m_righMotor.setFTerm(k_Fright);
    m_leftMotor.setFeedbackDevice(m_leftEncoder);
    m_righMotor.setFeedbackDevice(m_rightEncoder);
    m_leftMotor.setMaxSpeed(k_maxSpeed);
    m_righMotor.setMaxSpeed(k_maxSpeed);
    m_leftMotor.setPTerm(k_P);
    m_righMotor.setPTerm(k_P);
    m_leftMotor.setITerm(k_I);
    m_righMotor.setITerm(k_I);
    m_leftMotor.setIZone(k_IZone);
    m_righMotor.setIZone(k_IZone);
  }

  public void setPower(double leftPower, double rightPower) {
    m_leftMotor.setControlMode(SmartMotorMode.Power);
    m_righMotor.setControlMode(SmartMotorMode.Power);

    m_righMotor.set(rightPower);
    m_leftMotor.set(leftPower);
    
  }
  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_leftMotor.setControlMode(SmartMotorMode.Speed);
    m_righMotor.setControlMode(SmartMotorMode.Speed);

    m_leftMotor.set(leftSpeed);
    m_righMotor.set(rightSpeed);
  }

  public void yay() {}

 
 //Return the left encoder value
  public Encoder getLeftEncoder() {
    return (m_leftEncoder);
  }
 
  //Return the right encoder value
  public Encoder getRightEncoder() {
    return (m_rightEncoder);
  }
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
