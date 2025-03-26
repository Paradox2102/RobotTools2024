/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestRampShooter extends Command {
  private final ShooterSubsystem m_subsystem;
  private int m_power;

  /**
   * Creates a new TestRampShooter.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestRampShooter(ShooterSubsystem subsystem, int  Power) {
    Logger.log("TestRampShooter", 3, "TestRampShooter()");

    m_subsystem = subsystem;
    m_power = Power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestRampShooter", 2, "initialize()"); 
    //m_power = 0;
  
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestRampShooter", 3, String.format( "PWR: %f %d", m_power , m_subsystem.getShooterEncoder())); 
  
    m_subsystem.setShooterSpeed(m_power);
   // m_power += .01;
    
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestRampShooter", 2, String.format("end(%b)", interrupted));
    m_subsystem.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestRampShooter", -1, "isFinished()");
    return (m_power>1.1);
  }
}
