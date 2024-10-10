/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Encoder;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class CalibrateSpeedCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
  private double m_speed;

  /**
   * Creates a new CalibrateSpeedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CalibrateSpeedCommand(DriveSubsystem subsystem, double speed) {
    Logger.log("CalibrateSpeedCommand", 3, "CalibrateSpeedCommand()");

    m_subsystem = subsystem;
    m_leftEncoder = m_subsystem.getLeftEncoder();
    m_rightEncoder = m_subsystem.getRightEncoder();
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("CalibrateSpeedCommand", 2, "initialize()");
    Logger.log("CalibrateSpeedCommand", 0, ",Speed,Left,Right");
    m_subsystem.setSpeed(m_speed, m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("CalibrateSpeedCommand", -1, "execute()");
    Logger.log("TestMotorStringCommand", 0, String.format(",%f,%d,%d", m_speed * DriveSubsystem.k_maxSpeed, m_leftEncoder.getSpeed(), m_rightEncoder.getSpeed()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("CalibrateSpeedCommand", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("CalibrateSpeedCommand", -1, "isFinished()");
    return false;
  }
}
