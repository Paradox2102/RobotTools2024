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
public class TestDrive extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_FLModule;
  private final SwerveModule m_BLModule;
  private final SwerveModule m_BRModule;
  private final SwerveModule m_FRModule;
  private final double m_speed;

  /**
   * Creates a new TestDrive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestDrive(DriveSubsystem subsystem, double speed) {
    Logger.log("TestDrive", 3, "TestDrive()");

    m_subsystem = subsystem;
    m_FLModule = subsystem.getFLModule();
    m_BLModule = subsystem.getBLModule();
    m_BRModule = subsystem.getBRModule();
    m_FRModule = subsystem.getFRModule();
 
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestDrive", 2, "initialize()");
    m_FLModule.setTurnPosition(0);
    m_BLModule.setTurnPosition(0);
    m_BRModule.setTurnPosition(0);
    m_FRModule.setTurnPosition(0);
    
    // m_BRModule.setDrivePower(0.6);
    m_FLModule.setDriveSpeed(m_speed);
    m_BLModule.setDriveSpeed(m_speed);
    m_BRModule.setDriveSpeed(m_speed);
    m_FRModule.setDriveSpeed(m_speed);

    m_FLModule.resetDrivePosition(0);
    m_BLModule.resetDrivePosition(0);
    m_BRModule.resetDrivePosition(0);
    m_FRModule.resetDrivePosition(0);

    Logger.log("TestDrive", 1, ",Target,FLS,RLS,RRS,FRS,FLA,RLA,RRA,FRA,FLP,RLP,RRP,FRP");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestDrive", 1, String.format(",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
      m_speed, m_FLModule.getDriveSpeed(), m_BLModule.getDriveSpeed(), m_BRModule.getDriveSpeed(), m_FRModule.getDriveSpeed(),
        m_FLModule.getTurnPosition(), m_BLModule.getTurnPosition(), m_BRModule.getTurnPosition(), m_FRModule.getTurnPosition(),
        m_FLModule.getDrivePosition(), m_BLModule.getDrivePosition(), m_BRModule.getDrivePosition(), m_FRModule.getDrivePosition()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestDrive", 2, String.format("end(%b)", interrupted));
    m_FLModule.setDrivePower(0);
    m_BLModule.setDrivePower(0);
    m_BRModule.setDrivePower(0);
    m_FRModule.setDrivePower(0);

    m_FLModule.setTurnPower(0);
    m_BLModule.setTurnPower(0);
    m_BRModule.setTurnPower(0);
    m_FRModule.setTurnPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestDrive", -1, "isFinished()");
    return false;
  }
}
