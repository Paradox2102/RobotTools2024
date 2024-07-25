/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestRotation extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_module;
  private final double m_power;

  /**
   * Creates a new TestRotation.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestRotation(DriveSubsystem subsystem, double power) {
    Logger.log("TestRotation", 3, "TestRotation()");

    m_subsystem = subsystem;
    m_power = power;
    // m_module = subsystem.getFLModule();
    m_module = subsystem.getBLModule();
    // m_module = subsystem.getBRModule();
    // m_module = subsystem.getFRModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestRotation", 2, "initialize()");
    m_module.setTurnPower(m_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestRotation", -1, "execute()");
    // Logger.log("TestRotation", 1, String.format("a=%f", DriveSubsystem.normalizeAngle((m_module.getTurnPosition()))));
    Logger.log("TestRotation", 1, String.format(",%f", m_module.getTurnPosition()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestRotation", 2, String.format("end(%b)", interrupted));
    m_module.setTurnPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestRotation", -1, "isFinished()");
    return false;
  }
}
