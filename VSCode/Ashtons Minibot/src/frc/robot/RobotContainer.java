/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.CalibrateSpeedCommand;
import frc.robot.commands.CalibrateSpeedCommand;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.commands.DriveCourseCommand;
import frc.robot.commands.TestSpeedCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.DriveSubsystem;

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
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final CommandJoystick m_stick = new CommandJoystick(0);
  private final DriveCourseCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);

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
    m_DriveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_DriveSubsystem, m_stick));

    m_stick.button(1).onTrue(new DriveForTimeCommand(m_DriveSubsystem, 0.5, 5));
    m_stick.button(2).onTrue(new DriveForDistanceCommand(m_DriveSubsystem, 0.94, 30.0));
    m_stick.button(3).onTrue(new TestSpeedCommand(m_DriveSubsystem));
    m_stick.button(4).whileTrue(new CalibrateSpeedCommand(m_DriveSubsystem, 0.94));
    m_stick.button(5).onTrue(new TurnCommand(m_DriveSubsystem, 180, 0.60));
    m_stick.button(6).onTrue(new DriveCourseCommand(m_DriveSubsystem));
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
