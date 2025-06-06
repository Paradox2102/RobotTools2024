/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivesubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class drivefortimeCommand extends Command {
  private final drivesubsystem m_subsystem;

  /**
   * Creates a new drivefortimeCommand
   *.
   *
   * @param subsystem The subsystem used by this command.
   */
  public drivefortimeCommand
(drivesubsystem subsystem) {
    Logger.log("drivefortimeCommand", 3, "drivefortimeCommand()");

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("drivefortimeCommand", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("drivefortimeCommand", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("drivefortimeCommand", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("drivefortimeCommand", -1, "isFinished()");
    return false;
  }
}
