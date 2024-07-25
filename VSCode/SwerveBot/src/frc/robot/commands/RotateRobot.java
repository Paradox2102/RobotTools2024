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
public class RotateRobot extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_FLModule;
  private final SwerveModule m_RLModule;
  private final SwerveModule m_RRModule;
  private final SwerveModule m_FRModule;
  private final double m_speed;

  /**
   * Creates a new RotateRobot.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateRobot(DriveSubsystem subsystem, double speed) {
    Logger.log("RotateRobot", 3, "RotateRobot()");

    m_subsystem = subsystem;
    m_FLModule = subsystem.getFLModule();
    m_RLModule = subsystem.getBLModule();
    m_RRModule = subsystem.getBRModule();
    m_FRModule = subsystem.getFRModule();
 
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("RotateRobot", 2, "initialize()");
    m_FLModule.setTurnPosition(-45 + 90);
    m_RLModule.setTurnPosition(45 + 90);
    m_RRModule.setTurnPosition(-45 + 90);
    m_FRModule.setTurnPosition(45 + 90);
    
    m_FLModule.setDriveSpeed(-m_speed);
    m_RLModule.setDriveSpeed(-m_speed);
    m_RRModule.setDriveSpeed(m_speed);
    m_FRModule.setDriveSpeed(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("RotateRobot", 2, String.format("end(%b)", interrupted));
    m_FLModule.setDrivePower(0);
    m_RLModule.setDrivePower(0);
    m_RRModule.setDrivePower(0);
    m_FRModule.setDrivePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("RotateRobot", -1, "isFinished()");
    return false;
  }
}
