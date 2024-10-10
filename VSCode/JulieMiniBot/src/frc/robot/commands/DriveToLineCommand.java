/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.DigitalInput;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class DriveToLineCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private final DigitalInput m_irSensor;
  
  private static final double k_speed = 0.5;

  /**
   * Creates a new DriveToLineCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToLineCommand(DriveSubsystem subsystem, DigitalInput irSensor) {
    Logger.log("DriveToLineCommand", 3, "DriveToLineCommand()");

    m_subsystem = subsystem;
    m_irSensor = irSensor;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveToLineCommand", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("DriveToLineCommand", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("DriveToLineCommand", 2, String.format("end(%b)", interrupted));

    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("DriveToLineCommand", -1, "isFinished()");
    return m_irSensor.get();
  }
}