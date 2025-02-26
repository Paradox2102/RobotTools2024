/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class SetAngle extends Command {
  private final DriveSubsystem m_subsystem;
  private final double m_angle;

  /**
   * Creates a new Set270Deg.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetAngle(DriveSubsystem subsystem,double angle) {
    Logger.log("Set270Deg", 3, "Set270Deg()");
m_angle = angle;
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("Set270Deg", 2, "initialize()");
    m_subsystem.SwerveModuleGetFrontLeftModule().setSteeringPosition(m_angle);
    m_subsystem.SwerveModuleGetFrontRightModule().setSteeringPosition(m_angle);
    m_subsystem.SwerveModuleGetBackLeftModule().setSteeringPosition(m_angle);
    m_subsystem.SwerveModuleGetBackRightModule().setSteeringPosition(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("Set270Deg", -1, "execute()");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("Set270Deg", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("Set270Deg", -1, "isFinished()");
    return true;
  }
}
