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

  public static final double k_minPowerLeft = 0.27;
  public static final double k_minPowerRight = 0.27;
  public static final double k_maxSpeed = 1000;

  private static final double k_Fleft = 1 / k_maxSpeed;
  private static final double k_Fright = 0.95 / k_maxSpeed;
  private static final double k_P = 0.001; // Use the correct number for your robot
  private static final double k_I = 0.0005;
  private static final double k_IZone = 150;

  private PWMMotor m_leftMotor = new PWMMotor(k_leftMotorPWMPin, k_leftMotorDirPin);
  private PWMMotor m_rightMotor = new PWMMotor(k_rightMotorPWMPin, k_rightMotorDirPin);
  private Encoder m_leftEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_leftEncoderPin1,
      k_leftEncoderPin2);
  private Encoder m_rightEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_rightEncoderPin1,
      k_rightEncoderPin2);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
    Logger.log("DriveSubsystem", 1, String.format("LPWM=%d,LDIR=%d,RPWM=%d,RDIR=%d", k_leftMotorPWMPin, k_leftMotorDirPin, k_rightMotorPWMPin, k_rightMotorDirPin));

    m_leftMotor.setFeedbackDevice(m_leftEncoder);
    m_rightMotor.setFeedbackDevice(m_rightEncoder);
    m_leftMotor.setMaxSpeed(k_maxSpeed);
    m_rightMotor.setMaxSpeed(k_maxSpeed);

    m_leftMotor.setMinPower(k_minPowerLeft);
    m_rightMotor.setMinPower(k_minPowerRight);
    m_leftMotor.setFTerm(k_Fleft);
    m_rightMotor.setFTerm(k_Fright);
    m_leftMotor.setPTerm(k_P);
    m_rightMotor.setPTerm(k_P);
    m_leftMotor.setITerm(k_I);
    m_rightMotor.setITerm(k_I);
    m_leftMotor.setIZone(k_IZone);
    m_rightMotor.setIZone(k_IZone);

    m_rightEncoder.setInverted(true);
  }

  public void setPower(double leftPower, double rightPower) {
    // Logger.log("DriveSubsystem", 1, String.format("setPower(%f,%f)", leftPower, rightPower));
    m_leftMotor.setControlMode(SmartMotorMode.Power);
    m_rightMotor.setControlMode(SmartMotorMode.Power);

    m_rightMotor.set(rightPower);
    m_leftMotor.set(leftPower);
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_leftMotor.setControlMode(SmartMotorMode.Speed);
    m_rightMotor.setControlMode(SmartMotorMode.Speed);

    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
  }

  public Encoder getLeftEncoder() {
    return (m_leftEncoder);
  }

  public Encoder getRightEncoder() {
    return (m_rightEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
