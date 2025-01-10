package frc.robot.subsystems;
import robotCore.Encoder;
import robotCore.Gyro;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
    
    private final PWMMotor m_driveMotor;
    private final PWMMotor m_steeringMotor;
    private final Encoder m_driveEncoder;
    private final Encoder m_steeringEncoder;
    private final double k_degreesPerTick;

    public SwerveModule (int drivePWM, int driveDir, int driveEncInt, int driveEncDir, int steeringPWM, int steeringDir,
    int steeringEncA, int steeringEncB,  int i2cAddr, String name) {

        m_steeringMotor = new PWMMotor(steeringPWM, steeringDir, i2cAddr);
        m_driveMotor = new PWMMotor(drivePWM, driveDir, i2cAddr);

        m_steeringEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, steeringEncA, steeringEncB,i2cAddr, true);
        m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, driveEncInt, driveEncDir, i2cAddr, true);

        m_steeringMotor.setFeedbackDevice(m_steeringEncoder);
        m_driveMotor.setFeedbackDevice(m_driveEncoder);

        k_degreesPerTick = 360.0/m_steeringEncoder.getRange();
    }

    public void setPowerSteering(double power) {
        m_steeringMotor.setControlMode(SmartMotorMode.Power);
        m_steeringMotor.set(power);
    }

    public double getSteeringPosition() {

        return m_steeringEncoder.getPosition();
    }

    public void setSteeringMinPower (double minPower) {
        m_steeringMotor.setMinPower(minPower);
    }

    public void setSteeringZero (int zero) {
        m_steeringEncoder.setZero(zero);
    }

    public void setSteeringPValue (double m_p) {
        m_steeringMotor.setPTerm(m_p);
    }

    public void setSteeringIValue (double m_f) {
        m_steeringMotor.setITerm(m_f);
    }

    public void setSteeringDValue (double m_d) {
        m_steeringMotor.setDTerm(m_d);
    }

    public void setSteeringFValue (double m_f) {
        m_steeringMotor.setFTerm(m_f);
    }

    public double getSteeringPositionInDegrees() {
        return Gyro.normalizeAngle(getSteeringPosition() * k_degreesPerTick);
    }
}