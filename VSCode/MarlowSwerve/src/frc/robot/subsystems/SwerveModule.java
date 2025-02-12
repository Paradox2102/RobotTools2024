package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import robotCore.Encoder;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
    private final PWMMotor m_driveMotor;
    private final PWMMotor m_steeringMotor;
    private final Encoder m_driveEncoder;
    private final Encoder m_steeringEncoder;
    private double k_degreesPerTick;

    private String m_name;

    private double k_deadZone = 3;

    public SwerveModule(int drivePWM, int driveDir, int driveEncInt, int driveEncDir, int steeringPWM, int steeringDir,
            int steeringEncA, int steeringEncB, int i2cAddr, String name) {
        m_driveMotor = new PWMMotor(drivePWM, driveDir, i2cAddr);
        m_steeringMotor = new PWMMotor(steeringPWM, steeringDir, i2cAddr);
        m_steeringEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, steeringEncA, steeringEncB,
                i2cAddr, true);
        m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, driveEncInt, driveEncDir, i2cAddr, true);
        m_driveMotor.setFeedbackDevice(m_driveEncoder);
        m_steeringMotor.setFeedbackDevice(m_steeringEncoder);
        k_degreesPerTick = 360.0 / m_steeringEncoder.getRange();
        m_steeringMotor.setDeadZone(k_deadZone / k_degreesPerTick);
        m_name = name;

    }

    public void setSteeringPower(double power) {
        m_steeringMotor.setControlMode(SmartMotorMode.Power);
        m_steeringMotor.set(power);
    }

    public double getSteeringPos() {
        double steeringpos = m_steeringEncoder.getPosition();
        return steeringpos;
    }

    public void setSteeringMinPower(double power) {
        m_steeringMotor.setMinPower(power);
    }

    public void setDriveMaxSpeed(double speed) {
        m_driveMotor.setMaxSpeed(speed);
    }

    public void setDrivePower(double Power) {
        m_driveMotor.setControlMode(SmartMotorMode.Power);
        m_driveMotor.set(Power);
    }

    public void setDriveMinPower(double power) {
        m_driveMotor.setMinPower(power);
    }

    public void setZero(int zero) {
        m_steeringEncoder.setZero(zero);
    }

    public double getSteeringPosDeg() {

        return Gyro.normalizeAngle(getSteeringPos() * k_degreesPerTick);

    }

    public void setSteeringPTerm(double p) {
        m_steeringMotor.setPTerm(p);
    }

    public void setSteeringDTerm(double d) {
        m_steeringMotor.setDTerm(d);
    }

    public void setSteeringPosition(double angleInDegrees) {
        m_steeringMotor.setControlMode(SmartMotorMode.Position);
        m_steeringMotor.set(angleInDegrees / k_degreesPerTick);
    }

    public int getDriveEncoderS() {
        return m_driveEncoder.getSpeed();

    }

    public int getDriveEncoderP() {
        return m_driveEncoder.getPosition();
    }

    public void Speedset(double Speed) {
        m_driveMotor.setControlMode(SmartMotorMode.Speed);
        m_driveMotor.set(Speed);

    }

    public void setDrivePTerm(double p) {
        m_driveMotor.setPTerm(p);
    }

    public void setDriveITerm(double I) {
        m_driveMotor.setITerm(I);
    }

    public void setDriveFTerm(double f) {
        m_driveMotor.setFTerm(f);
    }

    public void setDriveIZone(double I) {
        m_driveMotor.setIZone(I);
    }

    public void setDriveSpeedInMetersPerSecond(double speed) {
        Speedset(speed * DriveSubsystem.k_ticksPerMeter);
    }

    public double getDriveSpeedInMetersPerSecond() {
        return getDriveEncoderS() / DriveSubsystem.k_ticksPerMeter;

    }

    public double getDrivePositionInMetersPerSecond() {
        return getDriveEncoderP() / DriveSubsystem.k_ticksPerMeter;
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        Rotation2d encoderRotation = Rotation2d.fromDegrees(getSteeringPosDeg());

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =  desiredState; //SwerveModuleState.optimize(desiredState, encoderRotation);

        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        // Logger.log(String.format("SwerveModule:%s", m_name), 1, String.format("angle:%f", state.angle.getDegrees()));
        Logger.log(String.format("SwerveModule:%s", m_name), 1, String.format("speed:%f", state.speedMetersPerSecond));

        setDriveSpeedInMetersPerSecond(state.speedMetersPerSecond);
        setSteeringPosition(state.angle.getDegrees());
    }

}
