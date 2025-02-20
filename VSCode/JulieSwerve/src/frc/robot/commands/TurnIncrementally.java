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
public class TurnIncrementally extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_angle;

  private SwerveModule m_swerve;
  private Timer m_timer;
  /**
   * Creates a new TurnIncrementally.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnIncrementally(DriveSubsystem subsystem, ModulePosition position) {
    Logger.log("TurnIncrementally", 3, "TurnIncrementally()");

    m_subsystem = subsystem;


    m_swerve = subsystem.getModule(position);
    m_timer = new Timer();
    m_angle = 0.0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnIncrementally", 2, "initialize()");

    m_swerve.setSteeringPosition(m_angle);

    m_timer.reset();
    m_timer.start();

    Logger.log("TurnIncrementally", 100, String.format("%f", (m_angle)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TurnIncrementally", -1, "execute()");

    double posInDegs = m_swerve.getSteeringPositionInDegrees();

    if (m_angle - posInDegs > 350) 
    {
        Logger.log("TurnIncrementally", 4, String.format("%f  :  %f", posInDegs, m_angle - posInDegs - 360));
    }
    else {
        Logger.log("TurnIncrementally", 4, String.format("%f  :  %f", posInDegs, m_angle - posInDegs));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TurnIncrementally", 2, String.format("end(%b)", interrupted));
    m_angle += 30.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TurnIncrementally", -1, "isFinished()");
    return m_timer.get() > 0.7;
  }
}