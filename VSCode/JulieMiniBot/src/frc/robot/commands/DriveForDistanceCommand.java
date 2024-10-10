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
 * A driving command using the driving subsystem.
 */
public class DriveForDistanceCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_speed = 0.0;
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
  private double m_distance;
  private static final double k_ticksPerInch = 2000.0 / 16.0;
  private static final double k_scale	= 0.005;

  /**
   * Creates a new DriveForDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForDistanceCommand(DriveSubsystem subsystem, double speed, double distance) {
    Logger.log("DriveForDistanceCommand", 3, "DriveForDistanceCommand()");

    m_subsystem = subsystem;
    m_speed = speed;
    m_leftEncoder = m_subsystem.getLeftEncoder();
    m_rightEncoder = m_subsystem.getRightEncoder();
    m_distance = distance * k_ticksPerInch;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveForDistanceCommand", 2, "initialize()");
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_subsystem.setSpeed(m_speed, m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("DriveForDistanceCommand", -1, "execute()");

    int leftDistance = m_leftEncoder.get();
    int rightDistance = m_rightEncoder.get();
    int deltaDistance = rightDistance - leftDistance;

    m_subsystem.setSpeed(m_speed + deltaDistance * k_scale, m_speed - deltaDistance * k_scale);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("DriveForDistanceCommand", 2, String.format("end(%b)", interrupted));
    m_subsystem.setPower(0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("DriveForDistanceCommand", -1, "isFinished()");
    return Math.abs(m_leftEncoder.get()) >= Math.abs(m_distance);
  }
}
