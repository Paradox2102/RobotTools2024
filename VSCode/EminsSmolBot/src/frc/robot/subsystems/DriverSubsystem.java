/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import java.io.PrintWriter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Encoder;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class DriverSubsystem extends SubsystemBase {

  private static final int k_leftMotorPWMPin = Device.M1_2_PWM;
  private static final int k_leftMotorDirPin = Device.M1_2_DIR;
  private static final int k_rightMotorPWMPin = Device.M1_1_PWM;
  private static final int k_rightMotorDirPin = Device.M1_1_DIR;

  private static final int k_leftEncoderPin1 = Device.Q1_INT;
  private static final int k_leftEncoderPin2 = Device.Q1_DIR;
  private static final int k_rightEncoderPin1 = Device.Q2_INT;
  private static final int k_rightEncoderPin2 = Device.Q2_DIR;

  private Encoder m_leftEncoder = new Encoder (robotCore.Encoder.EncoderType.Quadrature, k_leftEncoderPin1, k_leftEncoderPin2);
  private Encoder m_rightEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_rightEncoderPin1, k_rightEncoderPin2);

  private PWMMotor m_leftMotor = new PWMMotor(k_leftMotorPWMPin, k_leftMotorDirPin);
  private PWMMotor m_rightMotor = new PWMMotor(k_rightMotorPWMPin, k_rightMotorDirPin);

  private static final double k_minPowerLeft = 0.3;
  private static final double k_minPowerRight = 0.3;

  private static final double k_maxSpeed = 1075;

  private static final double k_Fleft = 1.025/k_maxSpeed;
  private static final double k_Fright = 0.745/k_maxSpeed;

  

  /**
   * Creates a new DriverSubsystem.
   */
  public DriverSubsystem() {
    Logger.log("DriverSubsystem", 3, "DriverSubsystem()");
    m_rightEncoder.setInverted(true);

    m_leftMotor.setMinPower(k_minPowerLeft);
    m_rightMotor.setMinPower(k_minPowerRight);
    

    m_leftMotor.setFTerm(k_Fleft);
    m_rightMotor.setFTerm(k_Fright);

    m_leftMotor.setMaxSpeed(k_maxSpeed);
    m_rightMotor.setMaxSpeed(k_maxSpeed);
  }

public double getMaxSpeed() {
  return k_maxSpeed;
}

  public void setPower(double leftPower, double rightPower)
  {
    m_leftMotor.setControlMode(SmartMotorMode.Power);


    m_leftMotor.set(leftPower);
    m_rightMotor.set(rightPower);
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_leftMotor.setControlMode(SmartMotorMode.Speed);
    m_rightMotor.setControlMode(SmartMotorMode.Speed);
 
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
  }

  public Encoder getLeftEncoder()
  {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder()
  {
    return m_rightEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriverSubsystem", -1, "periodic()");
  }
}
