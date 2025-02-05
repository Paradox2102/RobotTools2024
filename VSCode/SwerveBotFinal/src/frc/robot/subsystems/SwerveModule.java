package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import robotCore.Encoder;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
    private final static double k_deadZone = 3.0;

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

    public void setDriveSpeed(double speed) {
        m_driveMotor.setControlMode(SmartMotorMode.Speed);
        m_driveMotor.set(speed);
    }

    public void setDriveSpeedInMetersPerSecond(double speed) {
        setDriveSpeed(speed * DriveSubsystem.k_ticksPerMeter);
    }

    public double getDriveSpeedInMetersPerSecond() {
        return getDriveSpeed() / DriveSubsystem.k_ticksPerMeter;
    }

    public void setSteeringPower(double power) {
        // Logger.log("SwerveModule", 1, String.format("%s: setSteeringPower(%f)",
        // m_name, power));
        m_steeringMotor.setControlMode(SmartMotorMode.Power);
        m_steeringMotor.set(power);
    }

    public void setSteeringPosition(double angleInDegrees) {
        // Logger.log("SwerveModule", 1, String.format("%s: a=%f, set = %f", m_name,
        // angleInDegrees, angleInDegrees / k_degreesPerTick));
        m_steeringMotor.setControlMode(SmartMotorMode.Position);
        m_steeringMotor.set(angleInDegrees / k_degreesPerTick); // Convert to encoder units
    }

    public void setDriveMinPower(double power) {
        m_driveMotor.setMinPower(power);
    }

    public void setDriveF(double f) {
        Logger.log(String.format("SwerveModule-%s", m_name), 1, String.format("F=%f", f));
        m_driveMotor.setFTerm(f);
    }

    public void setDriveP(double p) {
        m_driveMotor.setPTerm(p);
    }

    public void setDriveI(double i) {
        m_driveMotor.setITerm(i);
    }

    public void setDriveIZone(double z) {
        m_driveMotor.setIZone(z);
    }

    public void resetDriveEncoder() {
        m_driveEncoder.reset();
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    public double getDrivePositionInMeters() {
        return getDrivePosition() / DriveSubsystem.k_ticksPerMeter;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionInMeters(), getSteeringPositionRotation2d());
    }

    public double getDriveSpeed() {
        return m_driveEncoder.getSpeed();
    }

    public double getSteeringPosition() {
        return m_steeringEncoder.getPosition();
    }

    public double getSteeringPositionInDegrees() {
        return Gyro.normalizeAngle(getSteeringPosition() * k_degreesPerTick);
    }

    public Rotation2d getSteeringPositionRotation2d() {
        return Rotation2d.fromDegrees(getSteeringPositionInDegrees());
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

    public void setSteeringMinPower(double power) {
        m_steeringMotor.setMinPower(power);
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

    public Rotation2d getSteeringRotation2d() {
        return Rotation2d.fromDegrees(getSteeringPositionInDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeedInMetersPerSecond(), getSteeringRotation2d());
    }

    public SwerveModuleState optimize(
        SwerveModuleState desiredState, Rotation2d currentAngle) {
      var delta = desiredState.angle.minus(currentAngle);
    //   Logger.log(String.format("SwerveModule-%s", m_name), 1, String.format("flip=%b,delta=%f,des=%f,cur=%f", Math.abs(delta.getDegrees())>90,delta.getDegrees(), desiredState.angle.getDegrees(), currentAngle.getDegrees()));
      if (Math.abs(delta.getDegrees()) > 90.0) {
        var ret = new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));

            // Logger.log(String.format("SwerveModule-%s", m_name), 1, String.format("Opt: ds=%f,rs=%f", desiredState.speedMetersPerSecond, ret.speedMetersPerSecond));
            return ret;
      } else {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        Rotation2d encoderRotation = getSteeringPositionRotation2d();

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimize(desiredState, encoderRotation);

        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        double da = state.angle.minus(encoderRotation).getDegrees();

        // if (Math.abs(da) > 90) {
        //     Logger.log(String.format("SwerveModule-%s", m_name), 1, String.format("ds=%f,ca=%f,da=%f",
        //             desiredState.angle.getDegrees(), encoderRotation.getDegrees(), da));
        // }

        // Logger.log(String.format("SwerveModule-%s", m_name), 1,
        //         String.format("s=%f,a=%f,da=%f,ca=%f,ds=%f", state.speedMetersPerSecond, state.angle.getDegrees(),
        //                 desiredState.angle.getDegrees(), encoderRotation.getDegrees(),
        //                 desiredState.speedMetersPerSecond));

        // System.out.print(String.format("%.2f,", state.speedMetersPerSecond));
        // if (m_name.equals("FrontRight")) {
        //     System.out.println(String.format("%.0f", desiredState.angle.getDegrees()));
        // }

        setDriveSpeedInMetersPerSecond(state.speedMetersPerSecond);
        setSteeringPosition(state.angle.getDegrees());

        // Robot.sleep(500);
    }
}
