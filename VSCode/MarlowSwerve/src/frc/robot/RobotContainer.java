/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwervePosConfig;
import frc.robot.commands.SwervePosConfig180;
import frc.robot.commands.SwervePosConfig270;
import frc.robot.commands.SwervePosConfig360;
import frc.robot.commands.SwervePosConfig90;
import frc.robot.commands.TestRamp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.ArcadeDrive;



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
  private final CommandJoystick m_Joystick = new CommandJoystick(0);
  private final ExampleCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, () -> m_Joystick.getX(),
    () -> -m_Joystick.getY(), () -> m_Joystick.getZ(), false));
 

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_Joystick.button(1).onTrue(new TestRamp(m_driveSubsystem));
    m_Joystick.button(2).onTrue(new SwervePosConfig(m_driveSubsystem));
    m_Joystick.button(3).onTrue(new SwervePosConfig90(m_driveSubsystem));
    m_Joystick.button(4).onTrue(new SwervePosConfig180(m_driveSubsystem));
    m_Joystick.button(5).onTrue(new SwervePosConfig270(m_driveSubsystem));
    m_Joystick.button(6).onTrue(new SwervePosConfig360(m_driveSubsystem));
   
    

    
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
