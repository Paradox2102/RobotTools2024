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
public class TestDriveSpeed extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_FLModule;
  private final Timer m_timer = new Timer();

  private static double m_power = 0.5;

  /**
   * Creates a new TestDriveSpeed.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestDriveSpeed(DriveSubsystem subsystem) {
    Logger.log("TestDriveSpeed", 3, "TestDriveSpeed()");

    m_subsystem = subsystem;
    m_FLModule = m_subsystem.getFrontLeftModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestDriveSpeed", 2, ",Power,Speed");
    m_FLModule.setDrivePower(m_power);
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestDriveSpeed", -1, "execute()");

    Logger.log("TestSterringMotor", 1, String.format(",%f,%f", m_power, m_FLModule.getDriveSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestDriveSpeed", 2, String.format("end(%b)", interrupted));

    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestDriveSpeed", -1, "isFinished()");

    return m_timer.get() >= 10;
  }
}
