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
public class PrintModuleAngles extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_FLmodule;
  private final SwerveModule m_BLmodule;
  private final SwerveModule m_BRmodule;
  private final SwerveModule m_FRmodule;
  private SwerveModule m_module;

  /**
   * Creates a new PrintModuleAngles.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PrintModuleAngles(DriveSubsystem subsystem) {
    Logger.log("PrintModuleAngles", 3, "PrintModuleAngles()");

    m_subsystem = subsystem;
    m_FLmodule = m_subsystem.getFrontLeftModule();
    m_BLmodule = m_subsystem.getBackLeftModule();
    m_BRmodule = m_subsystem.getBackRightModule();
    m_FRmodule = m_subsystem.getFrontRighModule();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("PrintModuleAngles", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("PrintModuleAngles", -1, "execute()");
    Logger.log("PrintModuleAngles", 1, String.format(",%f,%f,%f,%f", m_FLmodule.getSteeringPositionInDegrees(), m_BLmodule.getSteeringPositionInDegrees(), m_BRmodule.getSteeringPositionInDegrees(), m_FLmodule.getSteeringPositionInDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("PrintModuleAngles", 2, String.format("end(%b)", interrupted));
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("PrintModuleAngles", -1, "isFinished()");
    return false;
  }
}
