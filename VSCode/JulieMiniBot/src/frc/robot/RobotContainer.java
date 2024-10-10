/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Device;
import robotCore.DigitalInput;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.CalibrateSpeedCommand;
import frc.robot.commands.DriveCourseCommand;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.DriveToLineCommand;
import frc.robot.commands.EscapeCommand;
import frc.robot.commands.TestSpeedCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.DriveForDistanceCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  @SuppressWarnings("unused")
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final DigitalInput m_irSensor = new DigitalInput(Device.IO_4);
  private final CommandJoystick m_stick = new CommandJoystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, m_stick));

    m_stick.button(1).onTrue(new DriveForTimeCommand(m_driveSubsystem, 0.75, 1.0));
    m_stick.button(2).onTrue(new DriveForDistanceCommand(m_driveSubsystem, 1.0, -30.0));
    m_stick.button(3).onTrue(new TurnCommand(m_driveSubsystem, 1.0, 360.0));
    m_stick.button(4).onTrue(new DriveCourseCommand(m_driveSubsystem));
    m_stick.button(5).onTrue(new DriveToLineCommand(m_driveSubsystem, m_irSensor));
    m_stick.button(6).onTrue(new EscapeCommand(m_driveSubsystem, m_irSensor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
