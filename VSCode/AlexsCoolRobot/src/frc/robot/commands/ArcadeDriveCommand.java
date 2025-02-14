/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class ArcadeDriveCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private final CommandJoystick m_joystick;

  /**
   * Creates a new ArcadeDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param m_joystick2 
   */
  public ArcadeDriveCommand(DriveSubsystem subsystem, CommandJoystick joystick) {
    Logger.log("ArcadeDriveCommand", 3, "ArcadeDriveCommand()");

    m_joystick = joystick;

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDriveCommand", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("ArcadeDriveCommand", -1, "execute()");
    double y = m_joystick.getY();
    double x = m_joystick.getX();
    if (x < 0)
    {
      x = -x * x;
    }
    else
    {
      x = x * x;
    }
    if (y < 0)
    {
      y = -y * y;
    }
    else
    {
      y = y * y;
    }
  
    m_subsystem.setPower(y + x, y - x);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ArcadeDriveCommand", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("ArcadeDriveCommand", -1, "isFinished()");
    return false;
  }
}
