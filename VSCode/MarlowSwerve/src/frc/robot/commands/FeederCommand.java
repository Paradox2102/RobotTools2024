/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class FeederCommand extends Command {
  private final FeederSubsystem m_subsystem2;
  private final ShooterSubsystem m_subsystem1;
  private int m_Balls;
  private int m_ShooterSpeed;
  private int m_FeederSpeed;

  /**
   * Creates a new FeederCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FeederCommand(ShooterSubsystem subsystem1, FeederSubsystem subsystem2, int Balls, int FeederSpeed, int ShooterSpeed) {
    Logger.log("FeederCommand", 3, "FeederCommand()");

    m_subsystem1 = subsystem1;
    m_subsystem2 =  subsystem2;
    m_Balls = 2 * Balls;
    m_ShooterSpeed = ShooterSpeed;
    m_FeederSpeed = FeederSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("FeederCommand", 2, "initialize()");
    m_subsystem2.ballReset();
    
    m_subsystem2.setFeederSpeed(m_FeederSpeed);
    m_subsystem1.setShooterSpeed(m_ShooterSpeed);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("FeederCommand", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("FeederCommand", 2, String.format("end(%b)", interrupted));
    m_subsystem1.setShooterSpeed(0);
    m_subsystem2.setFeederSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("FeederCommand", 2, String.format("isFinished() : %d ", m_subsystem2.ballCount()));
    return (m_Balls <= m_subsystem2.ballCount());
  }
}
