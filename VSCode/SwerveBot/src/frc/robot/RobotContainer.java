/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.MaintainOrientationCommand;
import frc.robot.commands.SetModuleAngle;
import frc.robot.commands.SpinupCommand;
import frc.robot.commands.TestDrive;
import frc.robot.commands.TestDriveRamp;
import frc.robot.commands.TestRotation;
// import frc.robot.commands.TestRotationRamp;
import frc.robot.commands.TrackTargetCommand;
import frc.robot.commands.ArcadeDriveWpilib;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import robotCore.Device;
import robotCore.DigitalCounter;
import robotCore.apriltags.ApriltagLocation;
import robotCore.apriltags.ApriltagLocations;

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
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  private final CommandJoystick m_commandJoystick = new CommandJoystick(0);
  private final DigitalCounter m_ballCounter = new DigitalCounter(Device.IO_1);
  private final Joystick m_joystick = new Joystick(0);
  // private final XboxController m_controller = new XboxController(0);

  // private final PathPlannerAuto m_autoDriveStraight;
  private final PathPlannerAuto m_testAuto;

  private final ExampleCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);

    public static ApriltagLocation m_aprilTags[] = {
      new ApriltagLocation(1, 3, 3, 0),
      new ApriltagLocation(2, 3, 2.5, 90),
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    ApriltagLocations.setLocations(m_aprilTags);

    m_driveSubsystem.setDefaultCommand(
        new ArcadeDriveWpilib(m_driveSubsystem, () -> m_joystick.getX(),
            () -> -m_joystick.getY(),
            () -> m_joystick.getZ(), true));

    // m_driveSubsystem.setDefaultCommand(
    // new ArcadeDriveWpilib(m_driveSubsystem, () -> m_controller.getLeftX(),
    // () -> -m_controller.getLeftY(),
    // () -> m_controller.getRightX(), true));

    NamedCommands.registerCommand("Shoot10", new FeederCommand(m_FeederSubsystem, m_ballCounter, true, 10));
    NamedCommands.registerCommand("AutoAim2", new AutoAimCommand(m_driveSubsystem, 2));
    NamedCommands.registerCommand("Spinup", new SpinupCommand(m_shooterSubsystem));

    // m_autoDriveStraight = new PathPlannerAuto("Drive1Meter");
    m_testAuto = new PathPlannerAuto("TestAuto");
  
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_commandJoystick.button(1).whileTrue(new TestDrive(m_driveSubsystem, 0.4 *
    // DriveSubsystem.kMaxSpeed));
    m_commandJoystick.button(1).whileTrue(new FeederCommand(m_FeederSubsystem, m_ballCounter, true, 5));
    m_commandJoystick.button(2).toggleOnTrue(new SpinupCommand(m_shooterSubsystem));
    m_commandJoystick.button(3).toggleOnTrue(new TestRotation(m_driveSubsystem, 0.6));
    m_commandJoystick.button(4).whileTrue(new TestDrive(m_driveSubsystem, 2.0 * Constants.Drive.kMaxSpeed / 4));
    m_commandJoystick.button(5).onTrue(m_testAuto);
    m_commandJoystick.button(6).toggleOnTrue(new TestDriveRamp(m_driveSubsystem));

    m_commandJoystick.button(7).onTrue(new TrackTargetCommand(1));
    m_commandJoystick.button(8).onTrue(new MaintainOrientationCommand());

    m_commandJoystick.button(9).whileTrue(new SetModuleAngle(m_driveSubsystem, 0));
    m_commandJoystick.button(10).whileTrue(new SetModuleAngle(m_driveSubsystem, 90));
    m_commandJoystick.button(11).whileTrue(new SetModuleAngle(m_driveSubsystem, 180));
    m_commandJoystick.button(12).whileTrue(new SetModuleAngle(m_driveSubsystem, 270));
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
