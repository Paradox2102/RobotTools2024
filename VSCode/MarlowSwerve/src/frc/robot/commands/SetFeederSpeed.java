/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class SetFeederSpeed extends Command {
  private final FeederSubsystem m_subsystem;
  int m_Speed;

  /**
   * Creates a new SetFeederSpeed.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetFeederSpeed(FeederSubsystem subsystem, int Speed) {
    Logger.log("SetFeederSpeed", 3, "SetFeederSpeed()");

    m_subsystem = subsystem;
    m_Speed = Speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetFeederSpeed", 2, "initialize()");
    m_subsystem.setFeederSpeed(m_Speed);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("SetFeederSpeed", 5, String.format("speed: %d %d", m_Speed , m_subsystem.getFeederEnc()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SetFeederSpeed", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("SetFeederSpeed", -1, "isFinished()");
    return false;
  }
}
