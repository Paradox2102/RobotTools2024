package frc.robot;
import robotCore.Encoder;
import robotCore.Gyro;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
    private final PWMMotor m_driveMotor;
    private final PWMMotor m_steeringMotor;
    private final Encoder m_driveEncoder;
    private final Encoder m_steeringEncoder;
    private double k_degreesPerTick;
    private final double k_deadZone = 2.5;
    
    public SwerveModule(int drivePWM, int driveDir, int driveEncInt, int driveEncDir, int steeringPWM, int steeringDir,
    int steeringEncA, int steeringEncB,  int i2cAddr, String name) {
        m_driveMotor = new PWMMotor(drivePWM, driveDir, i2cAddr);
        m_steeringMotor = new PWMMotor(steeringPWM, steeringDir, i2cAddr);
        m_steeringEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, steeringEncA, steeringEncB, i2cAddr, true);
        m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, driveEncInt, driveEncDir, i2cAddr, true);
        m_driveMotor.setFeedbackDevice(m_driveEncoder);
        m_steeringMotor.setFeedbackDevice(m_steeringEncoder);
        k_degreesPerTick = 360.0 / m_steeringEncoder.getRange();
        m_steeringMotor.setDeadZone(k_deadZone / k_degreesPerTick);
    }
    
    public void setSteeringPower(double power) {
        m_steeringMotor.setControlMode(SmartMotorMode.Power);
        m_steeringMotor.set(power);
    }
    public void setDrivePower(double power) {
        m_driveMotor.setControlMode(SmartMotorMode.Power);
        m_driveMotor.set(power);
    }
    public double getSteeringPosition() {
        return m_steeringEncoder.getPosition();
    }
    public double getDrivingSpeed() {
        return m_driveEncoder.getSpeed();
    }
    public void setDrivingSpeed(double speed) {
        m_driveMotor.setControlMode(SmartMotorMode.Speed);
        m_driveMotor.set(speed);
    }
    public Encoder getSteeringEncoder() {
        return m_steeringEncoder;
    }
    public void setSteeringMinPower(double power) {
        m_steeringMotor.setMinPower(power);
    }
    public void setDriveMinPower(double power){
        m_driveMotor.setMinPower(power);
    }
    public void setDriveMinSpeed(double speed){
        m_driveMotor.setMinPower(speed);
    }
    public void setSteeringZero(int zero) {
        m_steeringEncoder.setZero(zero);
    }
    public void setSteeringPTerm(double p) {
        m_steeringMotor.setPTerm(p);
    }
    public void setSteeringDTerm(double d) {
        m_steeringMotor.setDTerm(d);
    }
    public void setDrivingPTerm(double p) {
        m_driveMotor.setPTerm(p);
    }
    public void setDrivingITerm(double i) {
        m_driveMotor.setITerm(i);
    }
    public void setDrivingFTerm(double f) {
        m_driveMotor.setITerm(f);
    }
    public void setDrivingIZone(double IZone){
        m_driveMotor.setIZone(IZone);
    }
    public void setSteeringPosition(double angleInDegrees) {
        m_steeringMotor.setControlMode(SmartMotorMode.Position);
        m_steeringMotor.set(angleInDegrees / k_degreesPerTick);     // Convert to encoder units
    }
    public double getSteeringPositionInDegrees() {
        return Gyro.normalizeAngle(getSteeringPosition() * k_degreesPerTick);
    }
    
}