package frc.robot;

import java.net.InetAddress;
import java.net.UnknownHostException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import robotCore.Device;
import robotCore.Encoder;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.Gyro;

public class Robot extends TimedRobot {
	@SuppressWarnings("unused")
	private RobotContainer m_robotContainer;
	Encoder m_encoder1;
	Encoder m_encoder2;
	Encoder m_encoder3;
	Encoder m_encoder4;
	PWMMotor m_motor1;
	PWMMotor m_motor2;
	Gyro m_gyro;


	public Robot(double period) {
		super(period);
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		Logger.log("Robot", 3, "robotInit()");
		m_robotContainer = new RobotContainer();

		InetAddress inetAddress;
		try {
			inetAddress = InetAddress.getLocalHost();
			System.out.println("IP Address:- " + inetAddress.getHostAddress());
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// m_motor1 = new PWMMotor(DriveSubsystem.FLDrivePWM, DriveSubsystem.FLDriveDir, DriveSubsystem.FLI2CAddr);
		// m_motor2 = new PWMMotor(DriveSubsystem.BLDrivePWM, DriveSubsystem.BLDriveDir, DriveSubsystem.BLI2CAddr);

		// m_motor1.setInverted(true);
		// m_motor1.setControlMode(SmartMotorMode.Power);
		// m_motor1.set(0.5);

		// m_motor2.setControlMode(SmartMotorMode.Power);
		// m_motor2.set(0.5);
		// m_motor1.enable();


		// Logger.setLogLevel("Encoder", 5);
		// Logger.setLogLevel("SwerveModule", 5);
		// Logger.setLogLevel("TwoWire", 5);

		// m_gyro = new Gyro();
		// m_encoder1 = new Encoder(EncoderType.Quadrature, DriveSubsystem.FLDriveEncInt, DriveSubsystem.FLDriveEncDir, true);
		// m_encoder2 = new Encoder(EncoderType.AnalogRotational, DriveSubsystem.FLTurnEncA, DriveSubsystem.FLTurnEncB, false);
		// m_encoder3 = new Encoder(EncoderType.Quadrature, DriveSubsystem.BLDriveEncInt, DriveSubsystem.BLDriveEncDir, true);
		// m_encoder4 = new Encoder(EncoderType.AnalogRotational, DriveSubsystem.BLTurnEncA, DriveSubsystem.BLTurnEncB, true);

		Device.startGroupRequests();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		Logger.log("Robot", -1, "robotPeriodic()");
		// Logger.log("Robot", 1, String.format("Ball count = %d", m_robotContainer.m_ballCounter.get()));
		// Logger.log("TwoWireLog", 1, "robotPeriodic()");

		// Logger.log("Robot", 1, String.format("Speed = %d", m_encoder1.getSpeed()));
		// Logger.log("Robot", 1, String.format("pos = %d", m_encoder2.getPosition()));
		
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		// long time = System.currentTimeMillis();
		commandScheduler();
		// Logger.log("Robot", 1, String.format("robotPeriodic: dt = %d", System.currentTimeMillis() - time));

		// throw new RuntimeException("Test");
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
		Logger.log("Robot", 2, "disabledInit()");
	}

	@Override
	public void disabledPeriodic() {
		Logger.log("Robot", -1, "disabledPeriodic()");
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		Logger.log("Robot", 2, "autonomousInit()");
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Logger.log("Robot", -1, "autonomousPeriodic()");
	}

	@Override
	public void teleopInit() {
		Logger.log("Robot", 2, "teleopInit()");
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Logger.log("Robot", -1, "teleopPeriodic()");
	}

	@Override
	public void testInit() {
		Logger.log("Robot", 2, "testInit()");
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		Logger.log("Robot", -1, "testPeriodic()");
	}
}
