/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.DriveSubsystem.ModulePosition;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TurnAllToCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_angle;

  private SwerveModule[] m_swerves = new SwerveModule[4];
  private Timer m_timer;
  /**
   * Creates a new TurnAllToCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnAllToCommand(DriveSubsystem subsystem, double angle) {
    Logger.log("TurnAllToCommand", 3, "TurnAllToCommand()");

    m_subsystem = subsystem;


    m_swerves[0] = m_subsystem.getModule(ModulePosition.FRONT_LEFT);
    m_swerves[1] = m_subsystem.getModule(ModulePosition.FRONT_RIGHT);
    m_swerves[2] = m_subsystem.getModule(ModulePosition.BACK_LEFT);
    m_swerves[3] = m_subsystem.getModule(ModulePosition.BACK_RIGHT);

    m_timer = new Timer();
    m_angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnAllToCommand", 2, "initialize()");

    for (int swerve_i = 0; swerve_i < 4; swerve_i++) {
      m_swerves[swerve_i].setSteeringPosition(m_angle);
    }

    m_timer.reset();
    m_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TurnAllToCommand", -1, "execute()");

    // double posInDegs = m_swerve.getSteeringPositionInDegrees();

    // if (m_angle - posInDegs > 350) 
    // {
    //     Logger.log("TurnAllToCommand", 4, String.format("%f  :  %f", posInDegs, m_angle - posInDegs - 360));
    // }
    // else {
    //     Logger.log("TurnAllToCommand", 4, String.format("%f  :  %f", posInDegs, m_angle - posInDegs));
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TurnAllToCommand", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TurnAllToCommand", -1, "isFinished()");
    return m_timer.get() > 0.7;
  }
}