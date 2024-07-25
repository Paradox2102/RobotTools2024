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
public class TestDriveRamp extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_FLModule;
  private final SwerveModule m_RLModule;
  private final SwerveModule m_RRModule;
  private final SwerveModule m_FRModule;
  private double m_power = 0;

  /**
   * Creates a new TestDriveRamp.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestDriveRamp(DriveSubsystem subsystem) {
    Logger.log("TestDriveRamp", 3, "TestDriveRamp()");

    m_subsystem = subsystem;
    m_FLModule = subsystem.getFLModule();
    m_RLModule = subsystem.getBLModule();
    m_RRModule = subsystem.getBRModule();
    m_FRModule = subsystem.getFRModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestDriveRamp", 2, "initialize()");
    m_FLModule.setTurnPosition(0);
    m_RLModule.setTurnPosition(0);
    m_RRModule.setTurnPosition(0);
    m_FRModule.setTurnPosition(0);

    Logger.log("TestDriveRamp", 1, ",Power,FL,RL,RR,FR");
    
    m_power = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestDrive", 1, String.format(",%f,%f,%f,%f,%f", m_power, m_FLModule.getDriveSpeed(), m_RLModule.getDriveSpeed(), m_RRModule.getDriveSpeed(), m_FRModule.getDriveSpeed()));
    m_FLModule.setDrivePower(m_power);
    m_RLModule.setDrivePower(m_power);
    m_RRModule.setDrivePower(m_power);
    m_FRModule.setDrivePower(m_power);

    m_power += 0.006;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestDriveRamp", 2, String.format("end(%b)", interrupted));
    m_FLModule.setDrivePower(0);
    m_RLModule.setDrivePower(0);
    m_RRModule.setDrivePower(0);
    m_FRModule.setDrivePower(0);

    m_FLModule.setTurnPower(0);
    m_RLModule.setTurnPower(0);
    m_RRModule.setTurnPower(0);
    m_FRModule.setTurnPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestDriveRamp", -1, "isFinished()");
    return m_power >= 1.2;
  }
}
