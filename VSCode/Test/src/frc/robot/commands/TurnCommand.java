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
public class TurnCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_speed;
  private double m_angle;
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
  private final double k_ticksPerDegree = 1850.0 / 360;

  /**
   * Creates a new TurnCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnCommand(DriveSubsystem subsystem, double speed, double angle) {
    Logger.log("TurnCommand", 3, "TurnCommand()");

    m_subsystem = subsystem;
    m_speed = speed;
    m_angle = angle * k_ticksPerDegree;
    m_leftEncoder = m_subsystem.getLeftEncoder();
    m_rightEncoder = m_subsystem.getRightEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnCommand", 2, "initialize()");

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    if (m_angle < 0)
    {
        m_subsystem.setSpeed(-m_speed, m_speed);    // Turn left
    }
    else
    {
        m_subsystem.setSpeed(m_speed, -m_speed);    // Turn right
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TurnCommand", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TurnCommand", 2, String.format("end(%b)", interrupted));

    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TurnCommand", -1, "isFinished()");

    int delta = m_leftEncoder.get() - m_rightEncoder.get();

    return (Math.abs(delta) >= Math.abs(m_angle));
  }
}