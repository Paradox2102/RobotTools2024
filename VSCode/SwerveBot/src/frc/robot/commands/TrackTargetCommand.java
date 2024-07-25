/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ArcadeDriveWpilib.DriveType;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TrackTargetCommand extends InstantCommand {
  private final int m_targetNo;

  /**
   * Creates a new TrackTargetCommand.
   */
  public TrackTargetCommand(int target) {
    Logger.log("TrackTargetCommand", 3, "TrackTargetCommand()");

    m_targetNo = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TrackTargetCommand", 2, "initialize()");

    ArcadeDriveWpilib.setDriveType(DriveType.TrackTarget, m_targetNo);
  }
}
