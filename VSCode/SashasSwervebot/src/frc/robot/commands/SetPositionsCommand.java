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
public class SetPositionsCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_position;
  private double m_timer;
  private Encoder m_steeringEncoderFL;
  private Encoder m_steeringEncoderBL;
  private Encoder m_steeringEncoderFR;
  private Encoder m_steeringEncoderBR;
  private int m_offset = 3;

  /**
   * Creates a new SetPositionsCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetPositionsCommand(DriveSubsystem subsystem, double Position) {
    Logger.log("SetPositionsCommand", 3, "SetPositionsCommand()");
    m_subsystem = subsystem;
    m_position=Position;
    m_steeringEncoderFL = m_subsystem.getFrontLeftModule().getSteeringEncoder();
    m_steeringEncoderBL = m_subsystem.getBackLeftModule().getSteeringEncoder();
    m_steeringEncoderFR = m_subsystem.getFrontRightModule().getSteeringEncoder();
    m_steeringEncoderBR = m_subsystem.getBackRightModule().getSteeringEncoder();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetPositionsCommand", 2, "initialize()");
    m_subsystem.getFrontLeftModule().setSteeringPosition(m_position);
    m_subsystem.getFrontRightModule().setSteeringPosition(m_position);
    m_subsystem.getBackLeftModule().setSteeringPosition(m_position);
    m_subsystem.getBackRightModule().setSteeringPosition(m_position);
    m_timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("SetPositionsCommand", -1, "execute()");
    Logger.log("testMotorSpeedCommand",0,String.format(",%f,%f,%f,%f", m_subsystem.getFrontRightModule().getSteeringPositionInDegrees(), m_subsystem.getFrontLeftModule().getSteeringPositionInDegrees(), m_subsystem.getBackLeftModule().getSteeringPositionInDegrees(), m_subsystem.getBackRightModule().getSteeringPositionInDegrees()));
    m_timer += 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SetPositionsCommand", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("SetPositionsCommand", -1, "isFinished()");
    return false;
  }
}
