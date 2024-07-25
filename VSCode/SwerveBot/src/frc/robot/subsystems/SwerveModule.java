package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import robotCore.Encoder;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
    private PWMMotor m_driveMotor;
    private PWMMotor m_turnMotor;
    private Encoder m_turnEncoder;
    public Encoder m_driveEncoder;
    private double m_turnZero;
@SuppressWarnings("unused")
    private String m_name;

    public static double k_angleRange = 4500;
    private static final double k_angleScale = k_angleRange / 360.0;
    // public static final double k_ticksPerMeter = 3649 / 1.397; //5842 / 1.45;
    

    public SwerveModule(int drivePWM, int driveDir, int driveEncInt, int driveEncDir, int turnPWM, int turnDir,
            int turnEncA, int turnEncB, int i2CAddr, double turnZero, String name) {
        Logger.log(String.format("SwerveModule-%s", name), 1, "SwerveModule()");
        Logger.log(String.format("SwerveModule-%s", name), 1, String.format("turnZero=%f", turnZero));
        m_driveMotor = new PWMMotor(drivePWM, driveDir, i2CAddr);
        m_turnMotor = new PWMMotor(turnPWM, turnDir, i2CAddr);
        m_turnEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, turnEncA, turnEncB, i2CAddr, true);
        m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, driveEncInt, driveEncDir, i2CAddr, true);
        m_name = name;

        m_turnZero = turnZero;

        // m_turnEncoder.setInverted(true);

        m_driveMotor.setFeedbackDevice(m_driveEncoder);
        m_turnMotor.setFeedbackDevice(m_turnEncoder);

        // m_turnMotor.setWrap(360.0 * k_angleScale);
        m_turnMotor.setDeadZone(2 * k_angleScale);
        setTurnPower(0);

        // Logger.log(String.format("SwerveModule-%s", m_name), 1, "Constructor Returns");
    }

    public void setDrivePower(double power) {
        m_driveMotor.setControlMode(SmartMotorMode.Power);

        m_driveMotor.set(power);
    }

    public void setDriveSpeed(double speed) {
        // Logger.log(String.format("SwerveModule-%s", m_name), 1, String.format("setDriveSpeed: %f", speed));
        if (speed == 0) {
            setDrivePower(0);
        } else {
            m_driveMotor.setControlMode(SmartMotorMode.Speed);

            m_driveMotor.set(speed);
        }
        // Logger.log(String.format("SwerveModule-%s", m_name), 1, String.format("setDriveSpeed - returns"));
    }

    public void setDriveSpeedMetersPerSecond(double speed)
    {
        setDriveSpeed(speed * Constants.Drive.kTicksPerMeter);
    }

    public void setTurnPower(double power) {
        m_turnMotor.setControlMode(SmartMotorMode.Power);

        m_turnMotor.set(power);
    }

    public void setTurnAndSpeed(double angle, double speed) {
        if (Math.abs(angle - getTurnPosition()) > 90) {
            angle = DriveSubsystem.normalizeAngle(angle + 180);
            speed = -speed;
        }

        setTurnPosition(angle);
        setDriveSpeed(speed);
    }

    public int getRawTurnPosition() {
        return m_turnEncoder.getPosition();
    }

    public int getRawTurnSpeed() {
        return m_turnEncoder.getSpeed();
    }

    public double getTurnPosition() {
        return DriveSubsystem.normalizeAngle((m_turnEncoder.getPosition() / k_angleScale) - m_turnZero);
    }

    public Rotation2d getTurnRotation2d() {
        return Rotation2d.fromDegrees(getTurnPosition());
    }

    public void resetDrivePosition(double pos) {
        m_driveEncoder.reset((int) pos);
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    public double getDrivePositionInMeters() {
        return m_driveEncoder.getPosition() / Constants.Drive.kTicksPerMeter;
    }

    public double getDriveSpeed() {
        return m_driveEncoder.getSpeed();
    }

    public double getDriveSpeedInMetersPerSecond() {
        return getDriveSpeed() / Constants.Drive.kTicksPerMeter;
    }

    public void setTurnPosition(double position) {
        // Logger.log("SwerveModule", 1, "setTurnPosition");
        m_turnMotor.setControlMode(SmartMotorMode.Position);

        m_turnMotor.set(DriveSubsystem.normalizeAngle(position + m_turnZero) * k_angleScale);
    }

    public void stop() {
        setDrivePower(0);
        setTurnPower(0);
    }

    public void setTurnPTerm(double p) {
        m_turnMotor.setPTerm(p);
    }

    public void setTurnITerm(double i) {
        m_turnMotor.setITerm(i);
    }

    public void setTurnDTerm(double d) {
        m_turnMotor.setDTerm(d);
    }

    public void setTurnIZone(double z) {
        m_turnMotor.setIZone(z);
    }

    public void setTurnMinPower(double p) {
        m_turnMotor.setMinPower(p);
    }

    public void setDriveMinPower(double p) {
        m_driveMotor.setMinPower(p);
    }

    public void setDriveMaxSpeed(double m) {
        m_driveMotor.setMaxSpeed(m);
    }

    public void setDriveFTerm(double f) {
        m_driveMotor.setFTerm(f);
    }

    public void setDrivePTerm(double p) {
        m_driveMotor.setPTerm(p);
    }

    public void setDriveITerm(double i) {
        m_driveMotor.setITerm(i);
    }

    public void setDriveDTerm(double d) {
        m_driveMotor.setDTerm(d);
    }

    public void setDriveIZone(double z) {
        m_driveMotor.setIZone(z);
    }

    // wpilib swerve

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionInMeters(), Rotation2d.fromDegrees(getTurnPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeedInMetersPerSecond(), getTurnRotation2d());           
      }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Logger.log(String.format("SwerveModule-%s", m_name), 1, "setDesiredState");

    var encoderRotation = Rotation2d.fromDegrees(getTurnPosition());

    // Logger.log(String.format("SwerveModule:%s", m_name), 1, String.format("angle=%f", m_turnEncoder.getPosition()));

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Logger.log(String.format("SwerveModule:%s", m_name), 1, String.format("d.speed=%f,d.angle=%f,c.angle=%f,s.speed=%f,s.angle=%f",
    //     desiredState.speedMetersPerSecond, desiredState.angle.getDegrees(), 
    //     encoderRotation.getDegrees(),
    //     state.speedMetersPerSecond, state.angle.getDegrees()));

    // Logger.log(String.format("SwerveModule:%s", m_name), 1, String.format("s=%f", state.speedMetersPerSecond));
    // Logger.log(String.format("SwerveModule-%s", m_name), 1, "setDesiredState - set speed");
    setDriveSpeedMetersPerSecond(state.speedMetersPerSecond);  // TODO - put back in
    // Logger.log(String.format("SwerveModule-%s", m_name), 1, "setDesiredState - set angle");
    setTurnPosition(state.angle.getDegrees());
    // Logger.log(String.format("SwerveModule-%s", m_name), 1, "setDesiredState - return");
  }
}
