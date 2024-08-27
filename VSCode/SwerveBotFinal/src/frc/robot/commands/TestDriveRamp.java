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
public class TestDriveRamp extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  private final SwerveModule m_frontRight;

  private double m_power;

  /**
   * Creates a new TestDriveRamp.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestDriveRamp(DriveSubsystem subsystem) {
    Logger.log("TestDriveRamp", 3, "TestDriveRamp()");

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
    Logger.log("TestDriveRamp", 2, "initialize()");
    Logger.log("TestDriveRamp", 1, ",power,FL,BL,BR,FR");
    m_power = 0;
    // m_frontLeft.setSteeringPosition(0);
    // m_backLeft.setSteeringPosition(0);
    // m_backRight.setSteeringPosition(0);
    // m_frontRight.setSteeringPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestDriveRamp", -1, "execute()");

    // m_power = 0.6;
    m_frontLeft.setDrivePower(m_power);
    m_backLeft.setDrivePower(m_power);
    m_backRight.setDrivePower(m_power);
    m_frontRight.setDrivePower(m_power);

    Logger.log("TestDriveRamp", 1, String.format(",%f,%f,%f,%f,%f", m_power, m_frontLeft.getDriveSpeed(),
        m_backLeft.getDriveSpeed(), m_backRight.getDriveSpeed(), m_frontRight.getDriveSpeed()));

    m_power += 0.006;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestDriveRamp", 2, String.format("end(%b)", interrupted));
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestDriveRamp", -1, "isFinished()");
    return m_power > 1.2;
  }
}
