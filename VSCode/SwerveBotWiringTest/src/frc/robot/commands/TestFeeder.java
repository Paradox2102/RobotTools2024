/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestFeeder extends Command {
  private final FeederSubsystem m_subsystem;

  /**
   * Creates a new TestFeeder.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestFeeder(FeederSubsystem subsystem) {
    Logger.log("TestFeeder", 3, "TestFeeder()");

    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestFeeder", 2, "initialize()");
    m_subsystem.setPower(0.35);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestFeeder", -1, "execute()");
    Logger.log("TestFeeder", 1, String.format(",%d", m_subsystem.getSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestFeeder", 2, String.format("end(%b)", interrupted));
    m_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestFeeder", -1, "isFinished()");
    return false;
  }
}
