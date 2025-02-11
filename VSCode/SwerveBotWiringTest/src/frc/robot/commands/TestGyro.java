/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestGyro extends Command {
  private final DriveSubsystem m_subsystem;

  /**
   * Creates a new TestGyro.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestGyro(DriveSubsystem subsystem) {
    Logger.log("TestGyro", 3, "TestGyro()");

    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestGyro", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestGyro", -1, "execute()");
    Logger.log("TestGyro", 1, String.format("yaw=%f", m_subsystem.getYaw()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestGyro", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestGyro", -1, "isFinished()");
    return false;
  }
}
