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


public class ShooterSubsystem extends SubsystemBase {
  
  private static final int k_PWMPin = Device.M3_2_PWM;
  private static final int k_DIRPin = Device.M3_2_DIR;
  private static final int k_EncIntPin = Device.Q4_INT;
  private static final int k_EncDirPin = Device.Q4_DIR;
  private static final int k_I2CAddr = 5;
  PWMMotor m_shootMotor = new PWMMotor(k_PWMPin, k_DIRPin, k_I2CAddr);

  Encoder m_shootEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_EncIntPin, k_EncDirPin);


  public static final double k_maxSpeed = 1585;
  public static final double k_f = .93 / k_maxSpeed;
  public static final double k_p = 0.0005;
  public static final double k_i = 0.001;
  public static final double k_iZone = 50;
 
  public ShooterSubsystem() {
    Logger.log("ShooterSubsystem", 3, "ShooterSubsystem()");

    m_shootEncoder.setInverted(true);
    m_shootMotor.setFeedbackDevice(m_shootEncoder);

   // m_shootMotor.setMaxSpeed(k_maxSpeed);
    m_shootMotor.setFTerm(k_f);
    m_shootMotor.setPTerm(k_p);
    m_shootMotor.setITerm(k_i);
    m_shootMotor.setIZone(k_iZone);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("ShooterSubsystem", -1, "periodic()");
  }

  public void setShooterSpeed(int Speed){
    m_shootMotor.setControlMode(SmartMotorMode.Speed);
    m_shootMotor.set(Speed);
 }


  public void setShooterPower(Double Power){
    m_shootMotor.setControlMode(SmartMotorMode.Power);
    m_shootMotor.set(Power);
 }
 public int getShooterEncoder(){
  return m_shootEncoder.getSpeed();
 }
 
}
