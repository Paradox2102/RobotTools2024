/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.DriveSubsystem.ModulePosition;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class GetZerosCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private final SwerveModule m_swerveFL;
  private final SwerveModule m_swerveFR;
  private final SwerveModule m_swerveBL;
  private final SwerveModule m_swerveBR;

  /**
   * Creates a new GetZerosCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GetZerosCommand(DriveSubsystem subsystem) {
    Logger.log("GetZerosCommand", 3, "GetZerosCommand()");

    m_subsystem = subsystem;

    m_swerveFL = m_subsystem.getModule(ModulePosition.FRONT_LEFT);
    m_swerveFR = m_subsystem.getModule(ModulePosition.FRONT_RIGHT);
    m_swerveBL = m_subsystem.getModule(ModulePosition.BACK_LEFT);
    m_swerveBR = m_subsystem.getModule(ModulePosition.BACK_RIGHT);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("GetZerosCommand", 2, "initialize()");

    Logger.log("GetZerosCommand", 4, String.format("FL: %f", m_swerveFL.getSteeringPosition()));
    Logger.log("GetZerosCommand", 4, String.format("FR: %f", m_swerveFR.getSteeringPosition()));
    Logger.log("GetZerosCommand", 4, String.format("BL: %f", m_swerveBL.getSteeringPosition()));
    Logger.log("GetZerosCommand", 4, String.format("BR: %f", m_swerveBR.getSteeringPosition()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("GetZerosCommand", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("GetZerosCommand", 2, String.format("end(%b)", interrupted));


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("GetZerosCommand", -1, "isFinished()");
    return true;
  }
}
