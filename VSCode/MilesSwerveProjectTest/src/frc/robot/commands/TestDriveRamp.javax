/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*--------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestDriveRamp extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_power;

  /**
   * Creates a new TestDriveRamp.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestDriveRamp(DriveSubsystem subsystem) {
    Logger.log("TestDriveRamp", 3, "TestDriveRamp()");

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
    m_power = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestDriveRamp", 2, "initialize()");
    m_subsystem.SwerveModuleGetBackLeftModule().setSteeringPosition(0);
    m_subsystem.SwerveModuleGetBackRightModule().setSteeringPosition(0);
    m_subsystem.SwerveModuleGetFrontLeftModule().setSteeringPosition(0);
    m_subsystem.SwerveModuleGetFrontRightModule().setSteeringPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestDriveRamp", -1, "execute()");
    Logger.log("GetMinSteeringPower", 0, String.format(",%f,%f,%f,%f,%f", m_power,
    m_subsystem.SwerveModuleGetBackLeftModule().getDriveMoterSpeed(), m_subsystem.SwerveModuleGetBackRightModule().getDriveMoterSpeed(), m_subsystem.SwerveModuleGetFrontRightModule().getDriveMoterSpeed(), m_subsystem.SwerveModuleGetFrontLeftModule().getDriveMoterSpeed()));

    m_subsystem.SwerveModuleGetBackLeftModule().setDriveMotorPower(m_power);
    m_subsystem.SwerveModuleGetBackRightModule().setDriveMotorPower(m_power);
    m_subsystem.SwerveModuleGetFrontLeftModule().setDriveMotorPower(m_power);
    m_subsystem.SwerveModuleGetFrontRightModule().setDriveMotorPower(m_power);
    m_power+=.0025;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestDriveRamp", 2, String.format("end(%b)", interrupted));
    m_subsystem.SwerveModuleGetBackLeftModule().setDriveMotorPower(0);
    m_subsystem.SwerveModuleGetBackRightModule().setDriveMotorPower(0);
    m_subsystem.SwerveModuleGetFrontLeftModule().setDriveMotorPower(0);
    m_subsystem.SwerveModuleGetFrontRightModule().setDriveMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestDriveRamp", -1, "isFinished()");
    return m_power>=1.2;
  }
}
