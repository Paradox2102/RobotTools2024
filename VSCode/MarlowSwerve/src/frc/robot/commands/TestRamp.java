/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestRamp extends Command {
  private final DriveSubsystem m_subsystem;
  private double power;

  /**
   * Creates a new TestRamp.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestRamp(DriveSubsystem subsystem) {
    Logger.log("TestRamp", 3, "TestRamp()");

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestRamp", 2, "initialize()"); 
    power = 500;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestRamp", 3, String.format( "PWR: %f %d %d %d %d",power , m_subsystem.getEncBL(), m_subsystem.getEncFR(), m_subsystem.getEncBR(),m_subsystem.getEncFL() )); 
  
    m_subsystem.setSpeed(power, power, power, power);
    m_subsystem.setRotation(0, 0, 0, 0);
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestRamp", 2, String.format("end(%b)", interrupted));
    m_subsystem.setSpeed(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestRamp", -1, "isFinished()");
    return (false);
  }
}
