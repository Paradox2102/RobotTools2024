/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class ArcadeDrive extends Command {
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;
  private final DoubleSupplier m_turn;
  public final static double k_deadZone = 0.1;
  private double m_angle = 0;

  /**
   * Creates a new ArcadeDrive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(DriveSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier turn) {
    Logger.log("ArcadeDrive", 3, "ArcadeDrive()");

    m_subsystem = subsystem;
    m_x = x;
    m_y = y;
    m_turn = turn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDrive", 2, "initialize()");
  }

  private double applyDeadZone(double value, double deadZone) {
    double v = Math.abs(value);
    if (v < deadZone) {
      return 0;
    }

    return ((v - deadZone) / (1 - deadZone))  * Math.signum(value);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_x.getAsDouble();
    double y = m_y.getAsDouble();
    double turn = m_turn.getAsDouble();

    x = applyDeadZone(x, 0.05);
    y = applyDeadZone(y, 0.05);
    // Logger.log("ArcadeDrive", 1, String.format("turn0=%f", turn));
    turn = applyDeadZone(turn, 0.20);

    boolean drive = (x != 0) || (y != 0);

    x = x * Math.abs(x);
    y = y * Math.abs(y);
    turn = turn * Math.abs(turn) * 0.5;

    // Logger.log("ArcadeDrive", 1, String.format("x=%f,y=%f,t=%f", x, y, turn));

    if ((x != 0) || (y != 0) || (turn != 0)) {
      double speed = 0;

      // Logger.log("ArcadeDrive", 1, String.format("turn1=%f", turn));

      if (drive) {
        m_angle = DriveSubsystem.normalizeAngle(Math.toDegrees(Math.atan2(-x, y))) + m_subsystem.getYaw();
        speed = Math.sqrt((x * x) + (y * y));
      }

      // Logger.log("ArcadeDrive", 1, String.format("x=%f,y=%f,t=%f,a=%f,s=%f", x, y, turn, m_angle, speed));

      m_subsystem.drive(m_angle, speed, turn);

      // Logger.log("ArcadeDrive", 1, String.format("angle=%f,power=%f", angle, speed * 0.5));
    }
    else {
      // Logger.log("ArcadeDrive", 1, "stop");
      m_subsystem.stop();
    }

    // RobotCoreBase.sleep(50);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ArcadeDrive", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("ArcadeDrive", -1, "isFinished()");
    return false;
  }
}
