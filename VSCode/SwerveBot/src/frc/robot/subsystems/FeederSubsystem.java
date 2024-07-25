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

public class FeederSubsystem extends SubsystemBase {
  private static final int k_PWMPin = Device.M3_1_PWM;
  private static final int k_DIRPin = Device.M3_1_DIR;
  private static final int k_encInt = Device.Q3_INT;
  private static final int k_encDir = Device.Q3_DIR;
  private static final int k_I2CAddr = 5;
  
  PWMMotor m_motor = new PWMMotor(k_PWMPin, k_DIRPin, k_I2CAddr);
  Encoder m_encoder = new Encoder(EncoderType.Quadrature, k_encInt, k_encDir);
  
  /**
   * Creates a new FeederSubsystem.
   */
  public FeederSubsystem() {
    Logger.log("FeederSubsystem", 3, "FeederSubsystem()");
    // m_motor.setInverted(true);
  }

  public void setPower(double power) {
    m_motor.setControlMode(SmartMotorMode.Power);
    m_motor.set(power);
  }

  public int getSpeed() {
    return m_encoder.getSpeed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("FeederSubsystem", -1, "periodic()");
  }
}
