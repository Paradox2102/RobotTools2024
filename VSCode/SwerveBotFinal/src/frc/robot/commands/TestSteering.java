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
public class TestSteering extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final SwerveModule m_frontRightModule;
  private double m_power = 0.5;

  /**
   * Creates a new TestSteering.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestSteering(DriveSubsystem subsystem) {
    Logger.log("TestSteering", 3, "TestSteering()");

    m_subsystem = subsystem;
    m_frontLeftModule = m_subsystem.getFrontLeftModule();
    m_backLeftModule = m_subsystem.getBackLeftModule();
    m_backRightModule = m_subsystem.getBackRightModule();
    m_frontRightModule = m_subsystem.getFrontRighModule();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestSteering", 2, "initialize()");
    Logger.log("TestSteering", 1, ",power,FL,BL,BR,FR");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestSteering", -1, "execute()");

    // Logger.log("TestRotationRamp", 1,
    //     String.format(",%f,%f,%f,%f,%f", m_power, m_frontLeftModule.getSteeringPosition(),
    //         m_backLeftModule.getSteeringPosition(), m_backRightModule.getSteeringPosition(),
    //         m_frontRightModule.getSteeringPosition()));

    Logger.log("TestRotationRamp", 1,
        String.format(",%f,%f,%f,%f,%f", m_power, m_frontLeftModule.getSteeringPositionInDegrees(),
            m_backLeftModule.getSteeringPositionInDegrees(), m_backRightModule.getSteeringPositionInDegrees(),
            m_frontRightModule.getSteeringPositionInDegrees()));
    m_frontLeftModule.setSteeringPower(m_power);
    m_backLeftModule.setSteeringPower(m_power);
    m_backRightModule.setSteeringPower(m_power);
    m_frontRightModule.setSteeringPower(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestSteering", 2, String.format("end(%b)", interrupted));
    m_frontLeftModule.setSteeringPower(0);
    m_backLeftModule.setSteeringPower(0);
    m_backRightModule.setSteeringPower(0);
    m_frontRightModule.setSteeringPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestSteering", -1, "isFinished()");
    return false;
  }
}
