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
import robotCore.Encoder.EncoderType;
import robotCore.SmartMotor.SmartMotorMode;

public class ShooterSubsystem extends SubsystemBase {
  private static final int k_PWMPin = Device.M3_2_PWM;
  private static final int k_DIRPin = Device.M3_2_DIR;
  private static final int k_EncIntPin = Device.Q4_INT;
  private static final int k_EncDirPin = Device.Q4_DIR;
  private static final int k_I2CAddr = 5;

  public static final double k_maxSpeed = 1600;
  public static final double k_f = 1.0 / k_maxSpeed;
  public static final double k_p = 0.002;
  public static final double k_i = 0.0005;
  public static final double k_iZone = 50;

  
  PWMMotor m_motor = new PWMMotor(k_PWMPin, k_DIRPin, k_I2CAddr);
  Encoder m_encoder = new Encoder(EncoderType.Quadrature, k_EncIntPin, k_EncDirPin);

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    Logger.log("ShooterSubsystem", 3, "ShooterSubsystem()");
    m_encoder.setInverted(true);

    m_motor.setFeedbackDevice(m_encoder);
    m_motor.setMaxSpeed(k_maxSpeed);
    m_motor.setFTerm(k_f);
    m_motor.setPTerm(k_p);
    m_motor.setITerm(k_i);
    m_motor.setIZone(k_iZone);
  }

  public void setPower(double power) {
    m_motor.setControlMode(SmartMotorMode.Power);
    m_motor.set(power);
  }

  public void setSpeed(double speed) {
    m_motor.setControlMode(SmartMotorMode.Speed);
    m_motor.set(speed);
    Logger.log("ShooterSubsystem", -1, "Somethin is wron with the SmartMotorMode()");
  }

  public Encoder getEncoder() {
    return m_encoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("ShooterSubsystem", -1, "periodic()");
  }
}
