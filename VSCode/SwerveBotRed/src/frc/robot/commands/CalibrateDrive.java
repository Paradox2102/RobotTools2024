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
public class CalibrateDrive extends Command {
  private final DriveSubsystem m_subsystem;
  private final double m_speed = 1500;
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  private final SwerveModule m_frontRight;

  /**
   * Creates a new CalibrateDrive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CalibrateDrive(DriveSubsystem subsystem) {
    Logger.log("CalibrateDrive", 3, "CalibrateDrive()");

    m_subsystem = subsystem;
    m_frontLeft = m_subsystem.getFrontLeftModule();
    m_backLeft = m_subsystem.getBackLeftModule();
    m_backRight = m_subsystem.getBackRightModule();
    m_frontRight = m_subsystem.getFrontRighModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("CalibrateDrive", 2, "initialize()");
    Logger.log("CalibrateDrive", 1, ",target,FL,BL,BR,FR");

    m_frontLeft.setSteeringPosition(0);
    m_backLeft.setSteeringPosition(0);
    m_backRight.setSteeringPosition(0);
    m_frontRight.setSteeringPosition(0);

    m_frontLeft.setDriveSpeed(m_speed);
    m_backLeft.setDriveSpeed(m_speed);
    m_backRight.setDriveSpeed(m_speed);
    m_frontRight.setDriveSpeed(m_speed);

    // m_frontLeft.setDrivePower(0.6);
    // m_backLeft.setDrivePower(0.6);
    // m_backRight.setDrivePower(0.6);
    // m_frontRight.setDrivePower(0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("CalibrateDrive", -1, "execute()");
    Logger.log("CalibrateDrive", 1, String.format(",%f,%f,%f,%f,%f", m_speed, m_frontLeft.getDriveSpeed(),
        m_backLeft.getDriveSpeed(), m_backRight.getDriveSpeed(), m_frontRight.getDriveSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("CalibrateDrive", 2, String.format("end(%b)", interrupted));
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("CalibrateDrive", -1, "isFinished()");
    return false;
  }
}
