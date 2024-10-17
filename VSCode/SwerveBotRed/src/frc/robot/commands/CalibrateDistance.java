/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class CalibrateDistance extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  private final SwerveModule m_frontRight;
  private final double m_speed = 1000;
  private final Timer m_timer = new Timer();
  private boolean m_run;

  /**
   * Creates a new CalibrateDistance.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CalibrateDistance(DriveSubsystem subsystem) {
    Logger.log("CalibrateDistance", 3, "CalibrateDistance()");

    m_subsystem = subsystem;
    m_frontLeft = m_subsystem.getFrontLeftModule();
    m_backLeft = m_subsystem.getBackLeftModule();
    m_backRight = m_subsystem.getBackRightModule();
    m_frontRight = m_subsystem.getFrontRighModule();

    m_timer.reset();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("CalibrateDistance", 2, "initialize()");

    m_frontLeft.setSteeringPosition(0);
    m_backLeft.setSteeringPosition(0);
    m_backRight.setSteeringPosition(0);
    m_frontRight.setSteeringPosition(0);

    m_frontLeft.setDriveSpeed(m_speed);
    m_backLeft.setDriveSpeed(m_speed);
    m_backRight.setDriveSpeed(m_speed);
    m_frontRight.setDriveSpeed(m_speed);

    m_frontLeft.resetDriveEncoder();
    m_backLeft.resetDriveEncoder();
    m_backRight.resetDriveEncoder();
    m_frontRight.resetDriveEncoder();

    m_run = true;

    Logger.resetElapsedTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("CalibrateDistance", -1, "execute()");
    Logger.log("CalibrateDistance", 1, String.format(",%f,%f,%f,%f", m_frontLeft.getDrivePosition(),
        m_backLeft.getDrivePosition(), m_backRight.getDrivePosition(), m_frontRight.getDrivePosition()));

    if (m_run && (m_frontLeft.getDrivePosition() >= 3500)) {
      Logger.log("CalibrateDistance", 1, "stop");
      m_run = false;
      m_subsystem.stop();
      m_timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("CalibrateDistance", 2, String.format("end(%b)", interrupted));
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("CalibrateDistance", -1, "isFinished()");
    return m_timer.get() > 0.2;
  }
}
