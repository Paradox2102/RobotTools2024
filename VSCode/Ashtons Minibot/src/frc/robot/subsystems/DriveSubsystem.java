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

  private PWMMotor m_leftMotor = new PWMMotor(k_leftMotorPWMPin, k_leftMotorDirPin );
  private PWMMotor m_rightMotor = new PWMMotor(k_rightMotorPWMPin, k_rightMotorDirPin);

    private Encoder m_leftEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_leftEncoderPin1, k_leftEncoderPin2);
    private Encoder m_rightEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_rightEncoderPin1, k_rightEncoderPin2);

    public static final double k_maxSpeed = 1000;
    private static final double k_leftMinPower = 0.3;
    private static final double k_rightMinPower = 0.3;
    private static final double k_FLeft =3.0/k_maxSpeed;
    private static final double k_FRight = 3.0/k_maxSpeed;
    private static final double k_PLeft = 0;
    private static final double k_PRight = 0;
    private static final double k_ILeft = 0;
    public static final double k_IRight = 0;
    public static final double k_DLeft = 0;
    public static final double k_DRight = 0;
    
  /**
   * Creates a new DriveSubsystem.
   */
    public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
    m_rightEncoder.setInverted(true);
    m_leftMotor.setMinPower(k_leftMinPower);
    m_rightMotor.setMinPower(k_rightMinPower);
    m_leftMotor.setMaxSpeed(k_maxSpeed);
    m_rightMotor.setMaxSpeed(k_maxSpeed);
    m_leftMotor.setFTerm(k_FLeft);
    m_rightMotor.setFTerm(k_FRight);
    m_leftMotor.setPTerm(k_PLeft);
    m_rightMotor.setPTerm(k_PRight);
    m_leftMotor.setITerm(k_ILeft);
    m_rightMotor.setITerm(k_IRight);
    m_leftMotor.setDTerm(k_DLeft);
    m_rightMotor.setDTerm(k_DRight);
    m_leftMotor.setFeedbackDevice(m_leftEncoder);
    m_rightMotor.setFeedbackDevice(m_rightEncoder);
  }

    public void setPower(double leftPower, double rightPower) {
    m_leftMotor.set(leftPower);
    m_rightMotor.set(rightPower);

    m_leftMotor.set(leftPower);
    m_rightMotor.set(rightPower);
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_leftMotor.setControlMode(SmartMotorMode.Power);
    m_rightMotor.setControlMode(SmartMotorMode.Power);

    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
  }
  public Encoder getLeftEncoder(){
    return m_leftEncoder;
  }

  public Encoder getRightEncoder(){
    return m_rightEncoder;
  }
  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
