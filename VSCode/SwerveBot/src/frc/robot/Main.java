package frc.robot;

// import robotCore.Device;

public class Main {
	static Robot m_robot;

	public static void main(String[] args) {
		// Device.PCTest = true;

		m_robot = new Robot(0.03);
		
		m_robot.startRobot();
	}

}
