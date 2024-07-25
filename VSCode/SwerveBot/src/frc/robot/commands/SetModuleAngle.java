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
public class SetModuleAngle extends Command {
  private final DriveSubsystem m_subsystem;
  private final double m_angle;
  private final SwerveModule m_module;
  // private final SwerveModule m_RLmodule;
  // private final SwerveModule m_RRmodule;
  // private final SwerveModule m_FRmodule;

  /**
   * Creates a new SetModuleAngle.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetModuleAngle(DriveSubsystem subsystem, double angle) {
    Logger.log("SetModuleAngle", 3, "SetModuleAngle()");

    m_subsystem = subsystem;
    m_angle = DriveSubsystem.normalizeAngle(angle);
    // m_module = m_subsystem.getFLModule();
    // m_module = m_subsystem.getBLModule();
    // m_module = m_subsystem.getBRModule();
    m_module = m_subsystem.getFRModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetModuleAngle", 2, "initialize()");
    m_module.setTurnPosition(m_angle);
    // m_RLmodule.setTurnPosition(m_angle);
    // m_RRmodule.setTurnPosition(m_angle);
    // m_FRmodule.setTurnPosition(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("SetModuleAngle", -1, "execute()");
    double angle = m_module.getTurnPosition();
    Logger.log("SetModuleAngle", 1, String.format(",%f,%f,%f", m_angle, angle, DriveSubsystem.normalizeAngle(m_angle - angle)));
    m_module.setTurnPosition(m_angle);
    // m_RLmodule.setTurnPosition(m_angle);
    // m_RRmodule.setTurnPosition(m_angle);
    // m_FRmodule.setTurnPosition(m_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SetModuleAngle", 2, String.format("end(%b)", interrupted));
    m_module.setTurnPower(0);
    // m_RLmodule.setTurnPower(0);
    // m_RRmodule.setTurnPower(0);
    // m_FRmodule.setTurnPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("SetModuleAngle", -1, "isFinished()");
    return false;
  }
}
