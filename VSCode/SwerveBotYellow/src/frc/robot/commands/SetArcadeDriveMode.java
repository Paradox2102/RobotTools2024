/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class SetArcadeDriveMode extends InstantCommand {
  ArcadeDrive.Mode m_mode;

  /**
   * Creates a new SetArcadeDriveMode.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetArcadeDriveMode(ArcadeDrive.Mode mode) {
    Logger.log("SetArcadeDriveMode", 3, "SetArcadeDriveMode()");

    m_mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetArcadeDriveMode", 2, "Track Target: " + m_mode);
    ArcadeDrive.setMode(m_mode);
  }
}
