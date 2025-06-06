/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
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
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandJoystick m_joystick = new CommandJoystick(0);

  private final ExampleCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);

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
      // m_controller.a().onTrue(new GetMinSteeringPower(m_DriveSubsystem));
      // m_controller.a().onTrue(new SetAngle(m_DriveSubsystem,0));
      // m_controller.b().onTrue(new SetAngle(m_DriveSubsystem,90));
      // m_controller.y().onTrue(new SetAngle(m_DriveSubsystem,180));
      // m_controller.x().onTrue(new CalibrateDrive(m_DriveSubsystem));
      m_joystick.button(1).whileTrue(new SetAngle(m_DriveSubsystem, 0));
      m_joystick.button(2).whileTrue(new SetAngle(m_DriveSubsystem, 90));
      m_joystick.button(3).whileTrue(new SetAngle(m_DriveSubsystem, 180));
      m_joystick.button(4).whileTrue(new SetAngle(m_DriveSubsystem, 270));
      
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
