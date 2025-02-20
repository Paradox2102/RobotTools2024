/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.ModulePosition;
import frc.robot.subsystems.SwerveModule;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class CalibrateDistanceCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_speed;
  private int m_ticks;
  
  private SwerveModule[] m_swerves = new SwerveModule[4];

  /**
   * Creates a new CalibrateDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CalibrateDistanceCommand(DriveSubsystem subsystem, double speed) {
    Logger.log("CalibrateDistanceCommand", 3, "CalibrateDistanceCommand()");
    Logger.log("CalibrateDistanceCommand", 0, ",Speed,FrontLeft,FrontRight,BackLeft,BackRight");
    m_subsystem = subsystem;
    m_speed = speed;
    m_ticks = 0;

    m_swerves[0] = m_subsystem.getModule(ModulePosition.FRONT_LEFT);
    m_swerves[1] = m_subsystem.getModule(ModulePosition.FRONT_RIGHT);
    m_swerves[2] = m_subsystem.getModule(ModulePosition.BACK_LEFT);
    m_swerves[3] = m_subsystem.getModule(ModulePosition.BACK_RIGHT);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("CalibrateDistanceCommand", 2, "initialize()");
    Logger.log("CalibrateDistanceCommand", 0, ",Speed,FrontLeft,FrontRight,BackLeft,BackRight");

    for (int swerve_i = 0; swerve_i < 4; swerve_i++) {
      m_swerves[swerve_i].setDriveSpeed(m_speed);
      m_swerves[swerve_i].setSteeringPosition(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("CalibrateDistanceCommand", -1, "execute()");
    Logger.log("CalibrateDistanceCommand", 0, String.format(",%f,%d,%d,%d,%d", m_speed * DriveSubsystem.k_maxDriveSpeed, 
      m_swerves[0].getDriveEncoder().getPosition(),
      m_swerves[1].getDriveEncoder().getPosition(),
      m_swerves[2].getDriveEncoder().getPosition(),
      m_swerves[3].getDriveEncoder().getPosition()
    ));

    m_ticks += 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("CalibrateDistanceCommand", 2, String.format("end(%b)", interrupted));

    for (int swerve_i = 0; swerve_i < 4; swerve_i++) {
      m_swerves[swerve_i].setDriveSpeed(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("CalibrateDistanceCommand", -1, "isFinished()");

    return false;
  }
}
