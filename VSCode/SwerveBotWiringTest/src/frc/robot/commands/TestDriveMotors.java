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
public class TestDriveMotors extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_FLModule;
  private final SwerveModule m_BLModule;
  private final SwerveModule m_BRModule;
  private final SwerveModule m_FRModule;

  private final static double k_time = 2.0;
  private final static double k_power = 1.0;
  private int m_state = 0;
  private final Timer m_timer = new Timer();


  /**
   * Creates a new TestDriveMotors.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestDriveMotors(DriveSubsystem subsystem) {
    Logger.log("TestDriveMotors", 3, "TestDriveMotors()");

    m_subsystem = subsystem;
    m_FLModule = m_subsystem.getFrontLeftModule();
    m_BLModule = m_subsystem.getBackLeftModule();
    m_BRModule = m_subsystem.getBackRightModule();
    m_FRModule = m_subsystem.getFrontRighModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestDriveMotors", 2, ",FL,BL,BR,FR");

    m_state = 0;
    m_timer.start();
    m_timer.reset();

    m_FLModule.setDrivePower(k_power);
    // m_BLModule.setDrivePower(k_power);
    // m_BRModule.setDrivePower(k_power);
    // m_FRModule.setDrivePower(k_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestDriveMotors", -1, "execute()");

    Logger.log("TestSterringMotor", 1, String.format(",%f,%f,%f,%f", m_FLModule.getDriveSpeed(), m_BLModule.getDriveSpeed(), m_BRModule.getDriveSpeed(), m_FRModule.getDriveSpeed()));

    if (m_timer.get() >= k_time) {
      m_subsystem.stop();

      m_state++;
      switch (m_state) {
        case 1:
          m_BLModule.setDrivePower(k_power);
          break;

        case 2:
          m_BRModule.setDrivePower(k_power);
          break;

        case 3:
          m_FRModule.setDrivePower(k_power);
          break;
      }

      m_timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestDriveMotors", 2, String.format("end(%b)", interrupted));

    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestDriveMotors", -1, "isFinished()");
    return m_state >= 4;
  }
}
