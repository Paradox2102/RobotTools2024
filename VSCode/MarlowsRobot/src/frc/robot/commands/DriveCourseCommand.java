package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;
 
public class DriveCourseCommand extends SequentialCommandGroup {
  /**
   * Creates a new NewCommand.
   */
  private final DriveSubsystem  m_subsystem;
 
  public DriveCourseCommand(DriveSubsystem subsystem) {
    Logger.log("DriveCourseCommand", 3, "DriveCourseCommand()");
 
    m_subsystem = subsystem;
    double turn90 = 80;
    double speed =  0.4;
    double wait = 0.5;
    double count = 0;
    while (count < 4) {
        addCommands( new DriveForDistanceCommand(subsystem, speed, 20),
     new WaitCommand(wait),
     new TurnCommand(subsystem, speed, turn90) );
     count += 1;
    }
      }
}