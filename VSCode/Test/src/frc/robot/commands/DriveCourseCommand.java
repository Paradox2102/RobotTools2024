package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

public class DriveCourseCommand extends SequentialCommandGroup {
  /**
   * Creates a new NewCommand.
   */
  private final DriveSubsystem m_subsystem;

  public DriveCourseCommand(DriveSubsystem subsystem) {
    Logger.log("DriveCourseCommand", 3, "DriveCourseCommand()");

    m_subsystem = subsystem;

    double turn90 = 85;
    double speed = 0.3;
    double wait = 0.5;
    
    addCommands(  new DriveForDistanceCommand(m_subsystem, speed, 20),
                  new WaitCommand(wait),
                  new TurnCommand(m_subsystem, speed, turn90),
                  new WaitCommand(wait),
                  new DriveForDistanceCommand(m_subsystem, speed, 10),
                  new WaitCommand(wait),
                  new TurnCommand(m_subsystem, speed, turn90),
                  new WaitCommand(wait),
                  new DriveForDistanceCommand(m_subsystem, speed, 20),
                  new WaitCommand(wait),
                  new TurnCommand(m_subsystem, speed, turn90),
                  new WaitCommand(wait),
                  new DriveForDistanceCommand(m_subsystem, speed, 10),
                  new WaitCommand(wait),
                  new TurnCommand(m_subsystem, speed, turn90));
  }
}