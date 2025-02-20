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
import frc.robot.commands.CalibrateDistanceCommand;
import frc.robot.commands.CalibrateDriveCommand;
import frc.robot.commands.GetZerosCommand;
import frc.robot.commands.TestDriveCommand;
import frc.robot.commands.TestTurnPowerCommand;
import frc.robot.commands.TurnAllToCommand;
import frc.robot.commands.TurnIncrementally;
import frc.robot.commands.TurnToCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.ModulePosition;

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

  private final CalibrateDriveCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);

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
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_controller.button(1).onTrue(new TestTurnPowerCommand(m_driveSubsystem, 0.75, 1.0));
    // m_controller.button(1).onTrue(new TestTurnPowerCommand(m_driveSubsystem, ModulePosition.FRONT_LEFT, 2.0));
    // m_xbox.a().onTrue(new TestTurnPowerCommand(m_driveSubsystem, ModulePosition.BACK_RIGHT));
    // m_xbox.leftBumper().onTrue(new GetZerosCommand(m_driveSubsystem));
    // m_xbox.povUp().onTrue(new TurnAllToCommand(m_driveSubsystem, 0));
    // m_xbox.povRight().onTrue(new TurnAllToCommand(m_driveSubsystem, 90));
    // m_xbox.povDown().onTrue(new TurnAllToCommand(m_driveSubsystem, 180));
    // m_xbox.povLeft().onTrue(new TurnAllToCommand(m_driveSubsystem, 270));    

    // m_xbox.rightBumper().onTrue(new TestDriveCommand(m_driveSubsystem));

    m_xbox.a().whileTrue(new CalibrateDriveCommand(m_driveSubsystem, 0.5));
    m_xbox.b().onTrue(new GetZerosCommand(m_driveSubsystem));
    m_xbox.x().onTrue(new TurnIncrementally(m_driveSubsystem, ModulePosition.BACK_LEFT));
    m_xbox.y().onTrue(new TurnAllToCommand(m_driveSubsystem, 0));
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
