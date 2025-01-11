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
public class SteeringTestCommand extends Command {

  private final DriveSubsystem m_subsystem;

  double m_power;
  SwerveModule m_frontLeftModule = DriveSubsystem.getFrontLeftModule();
  SwerveModule m_frontRightModule = DriveSubsystem.getFrontRightModule();
  SwerveModule m_backLeftModule = DriveSubsystem.getBackLeftModule();
  SwerveModule m_backRightModule = DriveSubsystem.getBackRightModule();
  Timer m_timer = new Timer();

  /**
   * Creates a new SteeringTestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SteeringTestCommand(DriveSubsystem subsystem) {
    Logger.log("SteeringTestCommand", 3, "SteeringTestCommand()");

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SteeringTestCommand", 2, "initialize()");
    m_power = 0.0;

    Logger.log(null, 0, ", Power,  FLM, FRM, BLM, BRM");
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Logger.log(null, 0, String.format(",%f, %f, %f,%f, %f", m_power, DriveSubsystem.getFrontLeftModule().getSteeringPosition(),DriveSubsystem.getFrontRightModule().getSteeringPosition(), DriveSubsystem.getBackLeftModule().getSteeringPosition(), DriveSubsystem.getBackRightModule().getSteeringPosition() ));

    m_frontLeftModule.setPowerSteering(DriveSubsystem.k_minPowerFL);
    m_frontRightModule.setPowerSteering(DriveSubsystem.k_minPowerFR);
    m_backLeftModule.setPowerSteering(DriveSubsystem.k_minPowerBL);
    m_backRightModule.setPowerSteering(DriveSubsystem.k_minPowerBR);

    
    m_power += 0.0025;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SteeringTestCommand", 2, String.format("end(%b)", interrupted));
    m_frontLeftModule.setPowerSteering(0);
    m_frontRightModule.setPowerSteering(0);
    m_backLeftModule.setPowerSteering(0);
    m_backRightModule.setPowerSteering(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("SteeringTestCommand", -1, "isFinished()");
    return m_power > 0.5;
  }
}
