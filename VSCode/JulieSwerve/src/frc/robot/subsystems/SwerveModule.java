package frc.robot.subsystems;

import robotCore.Encoder;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
    private final PWMMotor m_driveMotor;
    private final PWMMotor m_turnMotor;
    private final Encoder m_driveEncoder;
    private final Encoder m_turnEncoder;
    private final double k_degreesPerTick;

    public SwerveModule(int drivePWM, int driveDir, int driveEncInt, int driveEncDir, int turnPWM, int turnDir,
                       int turnEncA, int turnEncB,  int i2cAddr, String name) {
        m_driveMotor = new PWMMotor(
            drivePWM,
            driveDir,
            i2cAddr
        );

        m_turnMotor = new PWMMotor(
            turnPWM,
            turnDir,
            i2cAddr
        );

        m_driveEncoder = new Encoder(
            robotCore.Encoder.EncoderType.Quadrature,
            driveEncInt,
            driveEncDir,
            i2cAddr,
            true
        );

        m_turnEncoder = new Encoder(
            robotCore.Encoder.EncoderType.AnalogRotational,
            turnEncA,
            turnEncB,
            i2cAddr,
            true
        );


        
        m_driveMotor.setFeedbackDevice(m_driveEncoder);
        m_turnMotor.setFeedbackDevice(m_turnEncoder);

        k_degreesPerTick = 360.0 / m_turnEncoder.getRange();

        m_turnMotor.setDeadZone(2.0 / k_degreesPerTick);
    }

    
    public void setTurningPower(double power) {
        m_turnMotor.setControlMode(SmartMotorMode.Power);
        m_turnMotor.set(power);
    }

    public double getTurnPosition() {
        return m_turnEncoder.getPosition();
    }

    public void setSteeringMinPower(double minPower) {
        m_turnMotor.setMinPower(minPower);
    }

    public void setSteeringZero(int zero) {
        m_turnEncoder.setZero(zero);
    }

    public double getSteeringPositionInDegrees() {
        return Gyro.normalizeAngle(getTurnPosition() * k_degreesPerTick);
    }

    public void setSteeringPTerm(double p) {
        m_turnMotor.setPTerm(p);
    }

    public void setSteeringDTerm(double d) {
        m_turnMotor.setDTerm(d);
    }

    public void setSteeringPosition(double angleInDegrees) {
        // Logger.log("setSteeringPosition", 8, "init()");
        m_turnMotor.setControlMode(SmartMotorMode.Position);
        m_turnMotor.set(angleInDegrees / k_degreesPerTick);     // Convert to encoder units
    }
}