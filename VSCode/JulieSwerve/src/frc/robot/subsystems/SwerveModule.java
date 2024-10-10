package frc.robot.subsystems;

import robotCore.Encoder;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
    private final PWMMotor m_driveMotor;
    private final PWMMotor m_turnMotor;
    private final Encoder m_driveEncoder;
    private final Encoder m_turnEncoder;
    
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
    }

    
    public void setSteeringPower(double power) {
        m_turnMotor.setControlMode(SmartMotorMode.Power);
        m_turnMotor.set(power);
    }

    public double getSteerPosition() {
        return m_turnEncoder.getPosition();
    }
}