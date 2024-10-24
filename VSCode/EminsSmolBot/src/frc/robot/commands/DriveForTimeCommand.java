/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class DriveForTimeCommand extends Command {
  private final DriverSubsystem m_subsystem;
  Timer m_timer = new Timer();
  /**
   * Creates a new DriverSubsystem.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForTimeCommand(DriverSubsystem subsystem) {
    Logger.log("DriverSubsystem", 3, "DriverSubsystem()");

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveForTimeCommand", 2, "initialize()");
    m_timer.reset();
    m_timer.start();
    m_subsystem.setPower(1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("DriverSubsystem", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("DriverSubsystem", 2, String.format("end(%b)", interrupted));
    m_timer.stop();
    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("DriverSubsystem", -1, "isFinished()");
    return m_timer.get() > 5;
  }
}
