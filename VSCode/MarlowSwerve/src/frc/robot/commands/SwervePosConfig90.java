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
public class SwervePosConfig90 extends Command {
  private DriveSubsystem m_subsystem;

  

  /**
   * Creates a new SwervePosConfig90.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwervePosConfig90(DriveSubsystem subsystem) {
    Logger.log("SwervePosConfig90", 3, "SwervePosConfig90()");
 
    m_subsystem = subsystem;
    
  
    addRequirements(m_subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public  void initialize() {
    Logger.log("SwervePosConfig90", 1, "initialize()");
    m_subsystem.setRotation(90,90,90,90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("SwervePosConfig90", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SwervePosConfig90", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("SwervePosConfig90", -1, "isFinished()");
    return false;
  }
}
