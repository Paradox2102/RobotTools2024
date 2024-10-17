/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;
import robotCore.RobotCoreBase;

/**
 * An example command that uses an example subsystem.
 */
public class TurnToTarget extends Command {
  private final DriveSubsystem m_subsystem;
  private final int m_targetNo;
  private boolean m_onTarget;
  private final Timer m_timer = new Timer();
  private final double k_onTargetTime = 1.0;    // Must be on target for at least this amount of time

  /**
   * Creates a new TurnToTarget.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToTarget(DriveSubsystem subsystem, int targetNo) {
    Logger.log("TurnToTarget", 3, "TurnToTarget()");

    m_subsystem = subsystem;
    m_targetNo = targetNo;

    m_timer.start();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnToTarget", 2, "initialize()");

    m_onTarget = false;
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("AutoAimCommand", -1, "execute()");

    double targetAngle = m_subsystem.getTargetAngle(m_targetNo);
    double turnRate = m_subsystem.computeAutoAim(targetAngle);

    if (turnRate == 0) {
      if (m_timer.get() >= k_onTargetTime) {
        m_subsystem.stop();
        m_onTarget = true;
      }
    } else {
      m_subsystem.drive(0, 0, turnRate, true, ((TimedRobot) RobotCoreBase.getInstance()).getPeriod());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TurnToTarget", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TurnToTarget", -1, "isFinished()");
    return m_onTarget;
  }
}
