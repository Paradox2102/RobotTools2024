/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Logger;

public class ExampleSubsystem
 extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem
   * .
   */
  public ExampleSubsystem
  () {
    Logger.log("ExampleSubsystem", 3, "ExampleSubsystem()");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("ExampleSubsystem", -1, "periodic()");
  }
}
