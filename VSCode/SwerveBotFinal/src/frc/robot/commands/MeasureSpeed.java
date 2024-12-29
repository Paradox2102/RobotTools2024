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
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class MeasureSpeed extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_module;
  private final Timer m_timer = new Timer();

  /**
   * Creates a new MeasureSpeed.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MeasureSpeed(DriveSubsystem subsystem) {
    Logger.log("MeasureSpeed", 3, "MeasureSpeed()");

    m_subsystem = subsystem;
    m_module = m_subsystem.getFrontRighModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("MeasureSpeed", 2, "initialize()");

    m_module.setSteeringPower(0.5);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("MeasureSpeed", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("MeasureSpeed", 2, String.format("end(%b)", interrupted));
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("MeasureSpeed", -1, "isFinished()");
    return m_timer.get() >= 10;
  }
}
