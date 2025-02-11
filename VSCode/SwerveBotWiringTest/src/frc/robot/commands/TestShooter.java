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
public class TestShooter extends Command {
  private final ShooterSubsystem m_subsystem;

  /**
   * Creates a new TestShooter.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestShooter(ShooterSubsystem subsystem) {
    Logger.log("TestShooter", 3, "TestShooter()");

    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestShooter", 2, "initialize()");
    m_subsystem.setPower(0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestShooter", -1, "execute()");
    Logger.log("TestShooter", 1, String.format(",%f", m_subsystem.getSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestShooter", 2, String.format("end(%b)", interrupted));
    m_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestShooter", -1, "isFinished()");
    return false;
  }
}
