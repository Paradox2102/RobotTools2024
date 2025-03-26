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
public class testSpeedCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_power;
  private Encoder m_steeringEncoderFL;
  private Encoder m_steeringEncoderBL;
  private Encoder m_steeringEncoderFR;
  private Encoder m_steeringEncoderBR;

  /**
   * Creates a new testSpeedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public testSpeedCommand(DriveSubsystem subsystem, double power) {
    Logger.log("testSpeedCommand", 3, "testSpeedCommand()");
    m_subsystem = subsystem;
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
    Logger.log("testSpeedCommand", 2, "initialize()");
    Logger.log("testSpeedCommand",0,",Power, FL, BL, FR, BR");
    m_power = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("testSpeedCommand", -1, "execute()");
    Logger.log("testMotorSpeedCommand",0,String.format(",%f,%d,%d,%d,%d", m_power, m_steeringEncoderFL.getPosition(), m_steeringEncoderBL.getPosition(), m_steeringEncoderFR.getPosition(), m_steeringEncoderBR.getPosition()));

    m_subsystem.getFrontLeftModule().setSteeringPower(m_power);
    m_subsystem.getFrontRightModule().setSteeringPower(m_power);
    m_subsystem.getBackLeftModule().setSteeringPower(m_power);
    m_subsystem.getBackRightModule().setSteeringPower(m_power);
    m_power += 0.0025;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("testSpeedCommand", 2, String.format("end(%b)", interrupted));

    m_subsystem.getFrontLeftModule().setSteeringPower(0);
    m_subsystem.getFrontRightModule().setSteeringPower(0);
    m_subsystem.getBackLeftModule().setSteeringPower(0);
    m_subsystem.getBackRightModule().setSteeringPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("testSpeedCommand", -1, "isFinished()");
    return m_power >= 0.7;
  }
}
