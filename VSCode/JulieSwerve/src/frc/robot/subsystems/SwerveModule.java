package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    
    public void setSteeringPower(double power) {
        m_turnMotor.setControlMode(SmartMotorMode.Power);
        m_turnMotor.set(power);
    }

    public double getSteeringPosition() {
        return m_turnEncoder.getPosition();
    }

    public void setSteeringMinPower(double minPower) {
        m_turnMotor.setMinPower(minPower);
    }

    public void setSteeringZero(int zero) {
        m_turnEncoder.setZero(zero);
    }

    public double getSteeringPositionInDegrees() {
        return Gyro.normalizeAngle(getSteeringPosition() * k_degreesPerTick);
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

    public void setDriveSpeed(double speed) {
        m_driveMotor.setControlMode(SmartMotorMode.Speed);
        m_driveMotor.set(speed);
    }

    public Encoder getDriveEncoder() {
        return m_driveEncoder;
    }

    public void setDriveMinPower(double power) {
        m_driveMotor.setMinPower(power);
    }

    public void setDriveMaxSpeed(double speed) {
        m_driveMotor.setMaxSpeed(speed);
    }

    public void setDrivePTerm(double p) {
        m_driveMotor.setPTerm(p);
    }

    public void setDriveITerm(double i) {
        m_driveMotor.setITerm(i);
    }

    public void setDriveFTerm(double f) {
        m_driveMotor.setFTerm(f);
    }

    public void setDriveIZone(double zone) {
        m_driveMotor.setIZone(zone);
    }

    public void setDrivePower(double power) {
        m_driveMotor.setControlMode(SmartMotorMode.Power);
        m_driveMotor.set(power);
    }

    public void stop() {
        setDrivePower(0.0);
    }

    public void setDriveSpeedInMetersPerSecond(double speed) {
        setDriveSpeed(speed * DriveSubsystem.k_ticksPerMeter);
    }

    public double getDriveSpeed() {
        return m_driveEncoder.getSpeed();
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    public double getDriveSpeedInMetersPerSecond() {
        return getDriveSpeed() / DriveSubsystem.k_ticksPerMeter;
    }

    public double getDrivePositionInMetersPerSecond() {
        return getDrivePosition() / DriveSubsystem.k_ticksPerMeter;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d encoderRotation = Rotation2d.fromDegrees(getSteeringPosition());

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        setDriveSpeedInMetersPerSecond(state.speedMetersPerSecond);
        setSteeringPosition(state.angle.getDegrees());
    }
}