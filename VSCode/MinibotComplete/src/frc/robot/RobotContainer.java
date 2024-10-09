/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.CalibrateSpeedCommand;
import frc.robot.commands.DriveCourseCommand;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.DriveToLineCommand;
import frc.robot.commands.EscapeCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TestMotorSpeedCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import robotCore.Device;
import robotCore.DigitalInput;

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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final DigitalInput m_irSensor = new DigitalInput(Device.IO_4);
  private final CommandJoystick m_joystick = new CommandJoystick(0);
  private final CommandXboxController m_xbox = new CommandXboxController(0);

  private final ExampleCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, m_joystick));

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
    m_joystick.button(1).onTrue(new DriveForTimeCommand(m_driveSubsystem, 0.75, 3.0));
    m_joystick.button(2).onTrue(new DriveForDistanceCommand(m_driveSubsystem, 0.6, 30));
    m_joystick.button(3).onTrue(new TestMotorSpeedCommand(m_driveSubsystem));
    m_joystick.button(4).toggleOnTrue(new CalibrateSpeedCommand(m_driveSubsystem));
    m_joystick.button(5).onTrue(new TurnCommand(m_driveSubsystem, 0.5, -180));
    m_joystick.button(6).onTrue(new DriveCourseCommand(m_driveSubsystem));
    m_joystick.button(7).onTrue(new DriveToLineCommand(m_driveSubsystem, m_irSensor));
    m_joystick.button(8).onTrue(new EscapeCommand(m_driveSubsystem, m_irSensor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}