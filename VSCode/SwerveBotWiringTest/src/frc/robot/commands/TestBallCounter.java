/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestBallCounter extends Command {
  private final ShooterSubsystem m_subsystem;

  /**
   * Creates a new TestBallCounter.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestBallCounter(ShooterSubsystem subsystem) {
    Logger.log("TestBallCounter", 3, "TestBallCounter()");

    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestBallCounter", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestBallCounter", -1, "execute()");
    Logger.log("TestBallCounter", 1, String.format("count=%d", m_subsystem.getBallCount()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestBallCounter", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestBallCounter", -1, "isFinished()");
    return false;
  }
}
