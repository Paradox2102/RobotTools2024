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
import frc.robot.subsystems.DriveSubsystem.ModulePosition;
import frc.robot.subsystems.SwerveModule;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestTurnPowerCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_swerve;
  // private final Timer m_timer = new Timer();
  // private final double m_time;
  private double m_power = 0.0;

  /**
   * Creates a new TestTurnPowerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestTurnPowerCommand(DriveSubsystem subsystem, ModulePosition position) {
    Logger.log("TestTurnPowerCommand", 3, "TestTurnPowerCommand()");

    m_subsystem = subsystem;
    m_swerve = m_subsystem.getModule(position);
    // m_time = time;
    

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestTurnPowerCommand", 2, "initialize()");
    Logger.log("TestTurnPowerCommand", 0, ",Power,Position");

    // m_timer.reset();
    // m_timer.start();
    m_power = 0.0;

    m_swerve.setTurningPower(m_power);
    m_power += 0.0025;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestTurnPowerCommand", -1, "execute()");

    // Logger.log("TestTurnPowerCommand", 0, String.format(",%d,%d", 
    //                                                                 m_power,
    //                                                                 m_swerve.getSteerPosition()
    //                                                               )
    //           );

    // Logger.log("TestTurnPowerCommand", 0, String.format(",%f,%d", m_power, m_swerve.getSteerPosition()));
    Logger.log("TestMotorSpeedCommand", 0, String.format(",%f,%f", m_power, m_swerve.getTurnPosition()));
    
    m_swerve.setTurningPower(m_power);
    m_power += 0.0025;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestTurnPowerCommand", 2, String.format("end(%b)", interrupted));

    m_swerve.setTurningPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestTurnPowerCommand", -1, "isFinished()");
    return m_power >= 1.3;
  }
}
