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
public class TestDriveCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_speed;
  
  private SwerveModule[] m_swerves = new SwerveModule[4];

  /**
   * Creates a new TestDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestDriveCommand(DriveSubsystem subsystem) {
    Logger.log("TestDriveCommand", 3, "TestDriveCommand()");
    Logger.log("TestDriveCommand", 0, ",Speed,FrontLeft,FrontRight,BackLeft,BackRight");
    m_subsystem = subsystem;
    m_speed = 0.0125;

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
    Logger.log("TestDriveCommand", 2, "initialize()");
    Logger.log("TestDriveCommand", 0, ",Speed,FrontLeft,FrontRight,BackLeft,BackRight");
    
    m_speed = 0.0125;

    for (int swerve_i = 0; swerve_i < 4; swerve_i++) {
      m_swerves[swerve_i].setDriveSpeed(m_speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestDriveCommand", -1, "execute()");
    Logger.log("TestDriveCommand", 0, String.format(",%f,%d,%d,%d,%d", m_speed, 
      m_swerves[0].getDriveEncoder().getSpeed(),
      m_swerves[1].getDriveEncoder().getSpeed(),
      m_swerves[2].getDriveEncoder().getSpeed(),
      m_swerves[3].getDriveEncoder().getSpeed()
    ));

    m_speed += 0.0125;

    for (int swerve_i = 0; swerve_i < 4; swerve_i++) {
      m_swerves[swerve_i].setDriveSpeed(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestDriveCommand", 2, String.format("end(%b)", interrupted));

    for (int swerve_i = 0; swerve_i < 4; swerve_i++) {
      m_swerves[swerve_i].setDriveSpeed(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestDriveCommand", -1, "isFinished()");
    return m_speed > 1.2;
  }
}
