/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class DriveForTimeCommands extends Command {
  private final DriveSubsystem m_subsystem;
  private Timer m_timer = new Timer();
  private double m_power;
  private double m_time;

  /**
   * Creates a new DriveForTimeCommands.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForTimeCommands(DriveSubsystem subsystem, double power, double time) {
    Logger.log("DriveForTimeCommands", 3, "DriveForTimeCommands()");

    m_subsystem = subsystem;

    m_power = power;
    m_time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveForTimeCommands", 2, "initialize()");
     Logger.log("PowerCheck", 2, String.format("Power: %f", m_power));
      m_subsystem.setpower(m_power , m_power);
      m_timer.reset();
      m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("DriveForTimeCommands", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("DriveForTimeCommands", 2, String.format("end(%b)", interrupted));
    m_subsystem.setpower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("DriveForTimeCommands", -1, "isFinished()");
    return(m_timer.get() >= m_time);
  }
}
