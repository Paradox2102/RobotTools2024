/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.RobotCoreBase;

/**
 * An example command that uses an example subsystem.
 */
public class ArcadeDriveMaintainOrientation extends Command {
  public enum Mode {
    Normal,
    MaintainOrientation,
    TrackTarget
  }

  private final Mode m_mode;
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;
  private final DoubleSupplier m_turn;
  private final Boolean m_fieldRelative;
  public final static double k_deadZone = 0.05;
  public final static double k_turnDeadZone = 0.20;
  private double m_targetAngleInDegrees = 0;
  private final double k_p = 5.0/180.0;
  private final double k_angleDeadZone = 5;
  private boolean m_rotating = false;
  
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  
  /**
   * Creates a new ArcadeDrive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDriveMaintainOrientation(Mode mode, DriveSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier turn, boolean fieldRelative) {
    Logger.log("ArcadeDriveMaintainOrientation", 3, "ArcadeDriveMaintainOrientation()");

    m_mode = mode;
    m_subsystem = subsystem;
    m_x = x;
    m_y = y;
    m_turn = turn;
    m_fieldRelative = fieldRelative;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDriveMaintainOrientation", 2, "initialize()");

    m_targetAngleInDegrees = m_subsystem.getPose2d().getRotation().getDegrees();
    m_rotating = false;
  }

  private double maintainOrientation(double rot) {
    if (rot == 0) {
      if (m_rotating) {
        // Switching from manual to auto so save the current yaw
        m_rotating = false;
        m_targetAngleInDegrees = m_subsystem.getPose2d().getRotation().getDegrees();
      } else {
        // No manual rotation so calculate the rot need to maintain the current orientation
        Pose2d pos = m_subsystem.getPose2d();
      
        double da = Gyro.normalizeAngle(pos.getRotation().getDegrees() - m_targetAngleInDegrees);

        if (Math.abs(da) > k_angleDeadZone) {
          rot = -k_p * da;
        } else {
          rot = 0;
        }
      }
    }
    else {
      // Manual rotation so use the rot input from the joystick
      Logger.log("ArcadeDriveMaintainOrientation", 1, String.format("rot=%f", rot));
      m_rotating = true;
    }

    return rot;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_x.getAsDouble();
    double y = m_y.getAsDouble();
    double turn = m_turn.getAsDouble();

     // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(y, k_deadZone))
            * DriveSubsystem.k_maxSpeedMetersPerSecond;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(x, k_deadZone))
            * DriveSubsystem.k_maxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(turn, k_turnDeadZone))
            * DriveSubsystem.k_maxAngularSpeed;


    switch (m_mode) {
      case Normal:
        break;    // do nothing

      case MaintainOrientation:
        rot = maintainOrientation(rot);
        break;

      case TrackTarget:
        break;    // TODO
    }



    // Logger.log("ArcadeDriveMaintainOrientation", 1, String.format("x=%f,xs=%f,y=%f,ys=%f,t=%f,r=%f", x, xSpeed, y, ySpeed, turn, rot));
    m_subsystem.drive(xSpeed, ySpeed, rot, m_fieldRelative, ((TimedRobot) RobotCoreBase.getInstance()).getPeriod());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ArcadeDriveMaintainOrientation", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("ArcadeDriveMaintainOrientation", -1, "isFinished()");
    return false;
  }
}
