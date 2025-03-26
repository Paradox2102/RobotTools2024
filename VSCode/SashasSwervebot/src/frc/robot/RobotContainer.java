/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CalibrateDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FindSteeringZeroCommand;
import frc.robot.commands.SetPositionsCommand;
import frc.robot.commands.testDriveRampCommand;
import frc.robot.commands.testSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

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
  private final ExampleCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);
  private final CommandXboxController m_xbox = new CommandXboxController(0);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_xbox.leftBumper().onTrue(new CalibrateDriveCommand(m_driveSubsystem));
    m_xbox.y().onTrue(new SetPositionsCommand(m_driveSubsystem, 0));
    m_xbox.leftTrigger().onTrue(new SetPositionsCommand(m_driveSubsystem, 0));
    m_xbox.b().onTrue(new SetPositionsCommand(m_driveSubsystem, 90));
    m_xbox.x().onTrue(new SetPositionsCommand(m_driveSubsystem, 180));
    m_xbox.a().onTrue(new SetPositionsCommand(m_driveSubsystem, 270));
    m_xbox.rightBumper().onTrue(new testDriveRampCommand(m_driveSubsystem));
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
