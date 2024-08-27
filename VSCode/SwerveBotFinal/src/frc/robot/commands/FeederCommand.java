/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class FeederCommand extends Command {
  private final FeederSubsystem m_subsystem;
  private final double m_power = 0.4;
  private final static int k_stallSpeed = 100;
  private final static double k_stallTime = 0.1;
  private final static double k_reverseTime = 0.4;
  private final static double k_reversePower = 0.5;
  private boolean m_stalled = false;
  private final Timer m_timer = new Timer();
  private int m_count = 0;
  private final boolean m_reset;

  /**
   * Creates a new FeederCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FeederCommand(FeederSubsystem subsystem, boolean reset, int count) {
    Logger.log("FeederCommand", 3, String.format("FeederCommand(%d)", count));

    m_subsystem = subsystem;
    m_count = 2 * count;
    m_reset = reset;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("FeederCommand", 2, String.format("initialize: count=%d", m_count));
    m_stalled = false;
    m_subsystem.setPower(m_power);
    m_timer.reset();
    m_timer.start();

    if (m_reset) {
      m_subsystem.resetBallCounter();;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int speed = Math.abs(m_subsystem.getSpeed());

    Logger.log("FeederCommand", -1, "execute()");
    // Logger.log("FeederCommand", 1, String.format("speed=%d", speed));
    Logger.log("FeederCommand", 1, String.format("ball count = %d", m_subsystem.getBallCount()));

    double time = m_timer.get();

    if (m_stalled) {
      if (time >= k_reverseTime) {
        m_stalled = false;
        m_subsystem.setPower(m_power);
      }

    } else if (speed < k_stallSpeed) {
        if (time >= k_stallTime) {
          m_stalled = true;
          m_subsystem.setPower(-k_reversePower);
          m_timer.reset();
        }
    } else {
      m_timer.reset();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("FeederCommand", 2, String.format("end(%b)", interrupted));
    m_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("FeederCommand", -1, "isFinished()");

    if (m_count > 0) {
      return (m_subsystem.getBallCount() >= m_count);
    }

    return false;
  }
}
