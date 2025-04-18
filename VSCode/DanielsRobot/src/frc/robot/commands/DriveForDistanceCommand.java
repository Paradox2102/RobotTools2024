/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Encoder;
import robotCore.Logger;

/**
 * An DriveForTime command that uses an DriveForTime subsystem.
 */
public class DriveForDistanceCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_power;
  private double m_distance;
  private Encoder m_leftEncoder;
  /**
   * Creates a new DriveForDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForDistanceCommand(DriveSubsystem subsystem, double power, double distance) {
    Logger.log("DriveForDistanceCommand", 3, "DriveForDistanceCommand()");

    m_subsystem = subsystem;
    m_power = power;
    m_distance = distance;
    m_leftEncoder = subsystem.getLeftEncoder();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveForDistanceCommand", 2, "initialize()");
    
    m_subsystem.setPower(m_power,m_power);

    m_leftEncoder.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("DriveForDistanceCommand", -1, "execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("DriveForDistanceCommand", 2, String.format("end(%b)", interrupted));
    
    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("DriveForDistanceCommand", -1, "isFinished()");
    return (m_leftEncoder.get() >= m_distance);
  }
}
