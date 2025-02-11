/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestSteeringMotors extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_FLModule;

  private static double m_power = 0;


  /**
   * Creates a new TestSteeringMotors.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestSteeringMotors(DriveSubsystem subsystem) {
    Logger.log("TestSteeringMotors", 3, "TestSteeringMotors()");

    m_subsystem = subsystem;
    m_FLModule = m_subsystem.getFrontLeftModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestSteeringMotors", 2, ",Power,Angle");
    m_power = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestSteeringMotors", -1, "execute()");

    Logger.log("TestSterringMotor", 1, String.format(",%f,%f", m_power, m_FLModule.getSteeringPosition()));
   
    m_FLModule.setSteeringPower(m_power);
    m_power += 0.002;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestSteeringMotors", 2, String.format("end(%b)", interrupted));

    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestSteeringMotors", -1, "isFinished()");
    return m_power >= 1.2;
  }
}
