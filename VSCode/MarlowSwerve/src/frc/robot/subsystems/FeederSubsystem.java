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
import robotCore.DigitalCounter;
import edu.wpi.first.wpilibj.Timer;


public class FeederSubsystem extends SubsystemBase {
  private static final int k_PWMPin = Device.M3_1_PWM;
  private static final int k_DIRPin = Device.M3_1_DIR;
  private static final int k_encInt = Device.Q3_INT;
  private static final int k_encDir = Device.Q3_DIR;
  private static final int k_ballCounter = Device.IO_1;
  private static final int k_I2CAddr = 5;

  PWMMotor m_FeederMotor = new PWMMotor(k_PWMPin, k_DIRPin, k_I2CAddr);
  Encoder m_FeederEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_encInt, k_encDir);
  DigitalCounter m_FeederCounter = new DigitalCounter(k_ballCounter);

  private static final int k_maxSpeed = 2500;
  private static final double k_f = .97 / k_maxSpeed;
  private static final double k_p = 0.00007;
  private static final double k_i = 0.0002;
  private static final int k_iZone = 50;
  private final Timer m_stuckTimer = new Timer();
  private static final int TimerSeconds = 1;
  private int desiredSpeed = 0;
  private final Timer m_unstuckTimer = new Timer();
  

  public FeederSubsystem() {
    Logger.log("FeederSubsystem", 3, "FeederSubsystem()");
    m_stuckTimer.reset();
    m_stuckTimer.start();
    m_unstuckTimer.reset();
    
    m_FeederMotor.setFeedbackDevice(m_FeederEncoder);
    m_FeederMotor.setFTerm(k_f);
    m_FeederMotor.setPTerm(k_p);
    m_FeederMotor.setITerm(k_i);
    m_FeederMotor.setIZone(k_iZone);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("FeederSubsystem", -1, String.format("timer: %f , %d, %b", m_stuckTimer.get(), m_FeederEncoder.getSpeed(), (Math.abs(m_FeederEncoder.getSpeed()) > 100)));
    if (Math.abs(m_FeederEncoder.getSpeed()) > 100){
      m_stuckTimer.reset();
    }
    if((m_stuckTimer.get()> TimerSeconds) && (desiredSpeed != 0)){

      FeederStuck();
      m_unstuckTimer.start();
      
    }
    if(m_unstuckTimer.get() > TimerSeconds){
      setFeederSpeed(desiredSpeed);

    }

  }

  public void ballReset() {
     m_FeederCounter.reset();

  }


  public int ballCount() {
    return m_FeederCounter.get();

  }
 

  public void setFeederSpeed(int Speed) {
    m_FeederMotor.setControlMode(SmartMotorMode.Speed);
    m_FeederMotor.set(Speed);
    desiredSpeed = Speed;
    m_stuckTimer.restart();
    

  }

  
  public void FeederStuck() {
    m_FeederMotor.setControlMode(SmartMotorMode.Speed);
    m_FeederMotor.set(-1000);
   

  }

  public void setFeederPower(Double Power) {
    m_FeederMotor.setControlMode(SmartMotorMode.Power);
    m_FeederMotor.set(Power);
    desiredSpeed = (int)Math.round(Power * k_maxSpeed);
    m_stuckTimer.restart();

  }

  public int getFeederEnc() {
    return m_FeederEncoder.getSpeed();
  }

}
