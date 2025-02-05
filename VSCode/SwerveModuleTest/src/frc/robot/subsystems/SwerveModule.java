package frc.robot.subsystems;

import robotCore.Encoder;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
    private final static double k_deadZone = 2.0;

    private final PWMMotor m_driveMotor;
    private final PWMMotor m_steeringMotor;
    private final Encoder m_driveEncoder;
    private final Encoder m_steeringEncoder;
    private final double k_degreesPerTick;
    @SuppressWarnings("unused")
    private final String m_name;

    public SwerveModule(int drivePWM, int driveDir, int driveEncInt, int driveEncDir, int steeringPWM, int steeringDir,
            int steeringEncA, int steeringEncB, int i2cAddr, String name) {
        Logger.log("SwerveModule", 1, name);
        // Logger.log("SwerveModule", 1, String.format("%s: spwm=%d,sdir=%d", name,
        // steeringPWM, steeringDir));

        m_driveMotor = new PWMMotor(drivePWM, driveDir, i2cAddr);
        m_steeringMotor = new PWMMotor(steeringPWM, steeringDir, i2cAddr);
        m_steeringEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, steeringEncA, steeringEncB,
                i2cAddr, true);
        m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, driveEncInt, driveEncDir, i2cAddr, true);
        m_name = name;

        m_driveMotor.setFeedbackDevice(m_driveEncoder);
        m_steeringMotor.setFeedbackDevice(m_steeringEncoder);

        k_degreesPerTick = 360.0 / m_steeringEncoder.getRange();

        m_steeringMotor.setDeadZone(k_deadZone / k_degreesPerTick);
    }

    public void setDrivePower(double power) {
        m_driveMotor.setControlMode(SmartMotorMode.Power);
        m_driveMotor.set(power);
    }

    public void setSteeringPower(double power) {
        // Logger.log("SwerveModule", 1, String.format("%s: setSteeringPower(%f)",
        // m_name, power));
        m_steeringMotor.setControlMode(SmartMotorMode.Power);
        m_steeringMotor.set(power);
    }

    public void stopSteering() {
        setSteeringPower(0);
    }

    public void stopDrive() {
        setDrivePower(0);
    }

    public void stop() {
        stopSteering();
        stopDrive();
    }

    public double getDriveSpeed()
    {
        return m_driveEncoder.getSpeed();
    }

    public double getSteeringPosition()
    {
        return m_steeringEncoder.getPosition();
    }
}
