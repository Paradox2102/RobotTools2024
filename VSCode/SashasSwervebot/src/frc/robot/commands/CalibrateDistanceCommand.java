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
public class CalibrateDistanceCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_speed = 500;
  private double m_timer = 100;

  /**
   * Creates a new CalibrateDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CalibrateDistanceCommand(DriveSubsystem subsystem) {
    Logger.log("CalibrateDistanceCommand", 3, "CalibrateDistanceCommand()");

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("CalibrateDistanceCommand", 2, "initialize()");
    m_subsystem.getFrontLeftModule().setSteeringPosition(0);
    m_subsystem.getFrontRightModule().setSteeringPosition(0);
    m_subsystem.getBackLeftModule().setSteeringPosition(0);
    m_subsystem.getBackRightModule().setSteeringPosition(0);
    m_subsystem.getFrontLeftModule().setDrivePower(m_speed);
    m_subsystem.getFrontRightModule().setDrivePower(m_speed);
    m_subsystem.getBackRightModule().setDrivePower(m_speed);
    m_subsystem.getBackLeftModule().setDrivePower(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("CalibrateDistanceCommand", -1, "execute()");
    Logger.log("testMotorSpeedCommand",0,String.format(",%f,%f,%f,%f", m_subsystem.getFrontRightModule().getDrivePosition(), m_subsystem.getFrontLeftModule().getDrivePosition(), m_subsystem.getBackRightModule().getDrivePosition(), m_subsystem.getBackLeftModule().getDrivePosition()));
    m_timer -= 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("CalibrateDistanceCommand", 2, String.format("end(%b)", interrupted));
    m_subsystem.getFrontLeftModule().setDrivePower(0);
    m_subsystem.getFrontRightModule().setDrivePower(0);
    m_subsystem.getBackRightModule().setDrivePower(0);
    m_subsystem.getBackLeftModule().setDrivePower(0);
    m_timer = 100;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("CalibrateDistanceCommand", -1, "isFinished()");
    return m_timer <= 0;
  }
}
