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
  /**
   * Creates a new DriveSubsystem.
   */

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
  private PWMMotor m_rightMotor = new PWMMotor(k_rightMotorPWMPin, k_rightMotorDirPin);

  //Instantiate and assign Encoder objtects, left and right.
  private Encoder m_leftEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_leftEncoderPin1, k_leftEncoderPin2);
  private Encoder m_rightEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_rightEncoderPin1, k_rightEncoderPin2);

  public static final double k_maxSpeed = 1605.0;
  private static final double k_leftMinPower = 0.355;
  private static final double k_rightMinPower = 0.34;
  private static final double k_FLeft = 1.064/k_maxSpeed;
  private static final double k_FRight = 1.077/k_maxSpeed;
  
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
    m_rightEncoder.setInverted(true);
    
    m_leftMotor.setMinPower(k_leftMinPower);
    m_rightMotor.setMinPower(k_rightMinPower);
    
    m_leftMotor.setMaxSpeed(k_maxSpeed);
    m_rightMotor.setMaxSpeed(k_maxSpeed);

    m_leftMotor.setFTerm(k_FLeft);
    m_rightMotor.setFTerm(k_FRight);

    m_leftMotor.setFeedbackDevice(m_leftEncoder);
    m_rightMotor.setFeedbackDevice(m_rightEncoder);
  }

  // do the stuff things :3
  public void setPower(double leftPower, double rightPower) {
    m_leftMotor.setControlMode(SmartMotorMode.Power);
    m_rightMotor.setControlMode(SmartMotorMode.Power);

    m_leftMotor.set(leftPower);
    m_rightMotor.set(rightPower);
  }

  // nya owo uwu mrrrrrrp >/////<
  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_leftMotor.setControlMode(SmartMotorMode.Speed);
    m_rightMotor.setControlMode(SmartMotorMode.Speed);

    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
  }

  // get the left encoder
  public Encoder getLeftEncoder() {
    return (m_leftEncoder);
  }
 
  //Return the right encoder value
  public Encoder getRightEncoder() {
    return (m_rightEncoder);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
