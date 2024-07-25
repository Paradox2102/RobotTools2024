/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;
import robotCore.RobotCoreBase;

/**
 * An example command that uses an example subsystem.
 */
public class AutoAimCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private final int m_targetNo;

  /**
   * Creates a new AutoAimCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAimCommand(DriveSubsystem subsystem, int targetNo) {
    Logger.log("AutoAimCommand", 3, "AutoAimCommand()");

    m_subsystem = subsystem;
    m_targetNo = targetNo;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("AutoAimCommand", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("AutoAimCommand", -1, "execute()");

    double targetAngle = m_subsystem.getTargetAngle(m_targetNo);
    double rot = m_subsystem.computeAutoAim(targetAngle);

    m_subsystem.drive(0, 0, rot, true, ((TimedRobot) RobotCoreBase.getInstance()).getPeriod());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("AutoAimCommand", 2, String.format("end(%b)", interrupted));
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("AutoAimCommand", -1, "isFinished()");
    return false;
  }
}
