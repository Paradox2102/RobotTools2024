/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Logger;
import robotCore.PWMMotor;

public class drivesubsystem extends SubsystemBase {
  private int k_leftPWMPin = Device.M1_2_PWM;
  private int k_leftDIRPin = Device.M1_2_DIR;
  private int k_rightPWMPin = Device.M1_1_PWM;
  private int k_rightDIRPin = Device.M1_1_DIR;
  private PWMMotor m_leftMotor = new PWMMotor(k_leftPWMPin, k_leftDIRPin);
  private PWMMotor m_rightMotor = new PWMMotor(k_rightPWMPin, k_rightDIRPin);
  /**
   * Creates a new drivesubsystem.
   */
  public drivesubsystem() {
    Logger.log("drivesubsystem", 3, "drivesubsystem()");
  }
  
public void setPower (double leftPower, double rightPower) {
  m_leftMotor.set(leftPower);
  m_rightMotor.set(rightPower);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("drivesubsystem", -1, "periodic()");
  }
}
