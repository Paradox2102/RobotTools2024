/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import robotCore.Device;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.apriltags.ApriltagLocation;
import robotCore.apriltags.ApriltagLocations;
// import robotCore.Navigator;
import robotCore.apriltags.ApriltagsCamera;
import robotCore.apriltags.ApriltagsCamera.ApriltagsCameraType;
import robotCore.apriltags.PositionServer;

public class DriveSubsystem extends SubsystemBase {
  public static final int FLI2CAddr = 5;
  public static final int FLDrivePWM = Device.M1_1_PWM;
  public static final int FLDriveDir = Device.M1_1_DIR;
  public static final int FLTurnPWM = Device.M1_2_PWM;
  public static final int FLTurnDir = Device.M1_2_DIR;
  public static final int FLDriveEncInt = Device.Q1_INT;
  public static final int FLDriveEncDir = Device.Q1_DIR;
  public static final int FLTurnEncA = Device.A1_A;
  public static final int FLTurnEncB = Device.A1_B;

  public static final int BLI2CAddr = 5;
  public static final int BLDrivePWM = Device.M2_1_PWM;
  public static final int BLDriveDir = Device.M2_1_DIR;
  private static final int BLTurnPWM = Device.M2_2_PWM;
  private static final int BLTurnDir = Device.M2_2_DIR;
  public static final int BLDriveEncInt = Device.Q2_INT;
  public static final int BLDriveEncDir = Device.Q2_DIR;
  public static final int BLTurnEncA = Device.A2_A;
  public static final int BLTurnEncB = Device.A2_B;

  public static final int BRI2CAddr = 6;
  public static final int BRDrivePWM = Device.M1_1_PWM;
  public static final int BRDriveDir = Device.M1_1_DIR;
  public static final int BRTurnPWM = Device.M1_2_PWM;
  public static final int BRTurnDir = Device.M1_2_DIR;
  private static final int BRDriveEncInt = Device.Q1_INT;
  private static final int BRDriveEncDir = Device.Q1_DIR;
  private static final int BRTurnEncA = Device.A1_A;
  private static final int BRTurnEncB = Device.A1_B;

  private static final int FRI2CAddr = 6;
  private static final int FRDrivePWM = Device.M2_1_PWM;
  private static final int FRDriveDir = Device.M2_1_DIR;
  private static final int FRTurnPWM = Device.M2_2_PWM;
  private static final int FRTurnDir = Device.M2_2_DIR;
  private static final int FRDriveEncInt = Device.Q2_INT;
  private static final int FRDriveEncDir = Device.Q2_DIR;
  private static final int FRTurnEncA = Device.A2_A;
  private static final int FRTurnEncB = Device.A2_B;

  private static SwerveModule m_frontLeft; // = new SwerveModule(FLDrivePWM, FLDriveDir, FLDriveEncInt, FLDriveEncDir,
  // FLTurnPWM, FLTurnDir, FLTurnEncA, FLTurnEncB, FLI2CAddr, FLZeroPosition,
  // "FrontLeft");
  private static SwerveModule m_frontRight; // = new SwerveModule(FRDrivePWM, FRDriveDir, FRDriveEncInt, FRDriveEncDir,
  // FRTurnPWM, FRTurnDir, FRTurnEncA, FRTurnEncB, FRI2CAddr, FRZeroPosition,
  // "FrontRight");
  private static SwerveModule m_backLeft; // = new SwerveModule(BLDrivePWM, BLDriveDir, BLDriveEncInt, BLDriveEncDir,
  // BLTurnPWM, BLTurnDir, BLTurnEncA, BLTurnEncB, BLI2CAddr, BLZeroPosition,
  // "BackLeft");
  private static SwerveModule m_backRight; // = new SwerveModule(BRDrivePWM, BRDriveDir, BRDriveEncInt, BRDriveEncDir,
  // BRTurnPWM, BRTurnDir, BRTurnEncA, BRTurnEncB, BRI2CAddr, BRZeroPosition,
  // "BackRight");

  private static final String k_cameraIP = "127.0.1.1";
  private static final int k_cameraPort = 5800;

  // private static final Navigator m_navigator = new
  // Navigator(m_FRModule.m_driveEncoder, m_FLModule.m_driveEncoder);
  private final Gyro m_gyro = new Gyro();
  private ApriltagsCamera m_camera = new ApriltagsCamera();
  private final PositionServer m_posServer = new PositionServer(m_camera);

  // wpilib swerve

  public static final double k_maxSpeedMetersPerSecond = Constants.Drive.kMaxSpeed / Constants.Drive.kTicksPerMeter;
  public static final double k_maxAngularSpeed = 2; // Radians/second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.05842, 0.05842);
  private final Translation2d m_frontRightLocation = new Translation2d(0.05842, -0.05842);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.05842, 0.05842);
  private final Translation2d m_backRightLocation = new Translation2d(-0.05842, -0.05842);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };
  }

  // private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  // m_kinematics,
  // m_gyro.getRotation2d(),
  // getModulePositions());

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));

    setModuleStates(swerveModuleStates);
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
    // k_maxSpeedMetersPerSecond);
    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // m_backLeft.setDesiredState(swerveModuleStates[2]);
    // m_frontRight.setDesiredState(swerveModuleStates[1]);
    // m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  private Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  private Rotation2d getRotation2d() {
    Pose2d pose = m_poseEstimator.getEstimatedPosition();

    return pose.getRotation();
  }

  private SwerveDrivePoseEstimator m_poseEstimator;
  // wpilib swerve

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");

    m_frontLeft = new SwerveModule(FLDrivePWM, FLDriveDir, FLDriveEncInt, FLDriveEncDir,
        FLTurnPWM, FLTurnDir, FLTurnEncA, FLTurnEncB, FLI2CAddr, Constants.Drive.FLZeroPosition, "FrontLeft");
    m_frontRight = new SwerveModule(FRDrivePWM, FRDriveDir, FRDriveEncInt, FRDriveEncDir,
        FRTurnPWM, FRTurnDir, FRTurnEncA, FRTurnEncB, FRI2CAddr, Constants.Drive.FRZeroPosition, "FrontRight");
    m_backLeft = new SwerveModule(BLDrivePWM, BLDriveDir, BLDriveEncInt, BLDriveEncDir,
        BLTurnPWM, BLTurnDir, BLTurnEncA, BLTurnEncB, BLI2CAddr, Constants.Drive.BLZeroPosition, "BackLeft");
    m_backRight = new SwerveModule(BRDrivePWM, BRDriveDir, BRDriveEncInt, BRDriveEncDir,
        BRTurnPWM, BRTurnDir, BRTurnEncA, BRTurnEncB, BRI2CAddr, Constants.Drive.BRZeroPosition, "BackRight");

    m_gyro.invert(true);
    m_gyro.reset(0);

    m_frontLeft.setTurnMinPower(Constants.Drive.kFLTurnMinPower);
    m_frontLeft.setTurnPTerm(Constants.Drive.kFLTurnPTerm);
    m_frontLeft.setTurnDTerm(Constants.Drive.kFLTurnDTerm);
    // m_frontLeft.setTurnITerm(Constants.Drive.kFLTurnITerm);

    m_backLeft.setTurnMinPower(Constants.Drive.kBLTurnMinPower);
    m_backLeft.setTurnPTerm(Constants.Drive.kBLTurnPTerm);
    m_backLeft.setTurnDTerm(Constants.Drive.kBLTurnDTerm);

    m_backRight.setTurnMinPower(Constants.Drive.kBRTurnMinPower);
    m_backRight.setTurnPTerm(Constants.Drive.kBRTurnPTerm);
    m_backRight.setTurnDTerm(Constants.Drive.kBRTurnDTerm);

    m_frontRight.setTurnMinPower(Constants.Drive.kFRTurnMinPower);
    m_frontRight.setTurnPTerm(Constants.Drive.kFRTurnPTerm);
    m_frontRight.setTurnDTerm(Constants.Drive.kFRTurnDTerm);

    m_frontLeft.setDriveMinPower(Constants.Drive.FLDriveMinPower);
    m_frontLeft.setDriveFTerm(Constants.Drive.FLDriveFTerm);
    m_frontLeft.setDrivePTerm(Constants.Drive.kDrivePTerm);
    m_frontLeft.setDriveITerm(Constants.Drive.kDriveITerm);
    m_frontLeft.setDriveIZone(Constants.Drive.kDriveIZone);

    m_backLeft.setDriveFTerm(Constants.Drive.BLDriveFTerm);
    m_backLeft.setDrivePTerm(Constants.Drive.kDrivePTerm);
    m_backLeft.setDriveITerm(Constants.Drive.kDriveITerm);
    m_backLeft.setDriveIZone(Constants.Drive.kDriveIZone);

    m_backRight.setDriveMinPower(Constants.Drive.BRDriveMinPower);
    m_backRight.setDriveFTerm(Constants.Drive.BRDriveFTerm);
    m_backRight.setDrivePTerm(Constants.Drive.kDrivePTerm);
    m_backRight.setDriveITerm(Constants.Drive.kDriveITerm);
    m_backRight.setDriveIZone(Constants.Drive.kDriveIZone);

    m_frontRight.setDriveMinPower(Constants.Drive.FRDriveMinPower);
    m_frontRight.setDriveFTerm(Constants.Drive.FRDriveFTerm);
    m_frontRight.setDrivePTerm(Constants.Drive.kDrivePTerm);
    m_frontRight.setDriveITerm(Constants.Drive.kDriveITerm);
    m_frontRight.setDriveIZone(Constants.Drive.kDriveIZone);

    m_camera.setCameraInfo(0, 5, 0, ApriltagsCameraType.PiCam_640x480);
    m_camera.connect(k_cameraIP, k_cameraPort); // TODO - testing
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getGyroRotation2d(), getModulePositions(),
        new Pose2d(3, 8.2 / 2, Rotation2d.fromDegrees(0)));
    m_posServer.start();

    // Pathplanner

    AutoBuilder.configureHolonomic(this::getPose2d,
        this::ResetPose,
        this::getChassisSpeeds,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1, 0, 0),
            new PIDConstants(1, 0, 0),
            k_maxSpeedMetersPerSecond,
            (3.2569 * 2.54) / 100,
            new ReplanningConfig()),
        // new ReplanningConfig(),
        () -> false,
        this);
  }

  private void ResetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
        m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
        m_backRight.getState() };
    return states;
  }

  private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(m_kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public Pose2d getPose2d() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public double getYaw() {
    // return m_navigator.getYaw();
    return 0;
  }

  /*
   * Return the angle to the specified Apriltags target
   */
  public double getTargetAngle(int target) {
    ApriltagLocation location = ApriltagLocations.findTag(target);

    if (location == null) {
      throw new RuntimeException(String.format("Invalid target: %d", target));
    }

    Pose2d pose = getPose2d();
    double dx = location.m_xMeters - pose.getX();
    double dy = location.m_yMeters - pose.getY();

    return Math.toDegrees(Math.atan2(dy, dx));
  }

  private final double k_angleDeadZone = 3;
  private final double k_p = 10.0 / 180.0;

  /*
   * Compute the rotation rate needed to perform a turn to target maneuver
   */
  public double computeAutoAim(double targetAngleInDegrees) {
    Pose2d pos = getPose2d();

    double da = Gyro.normalizeAngle(pos.getRotation().getDegrees() - targetAngleInDegrees);

    if (Math.abs(da) > k_angleDeadZone) {
      return -k_p * da;
    }

    return 0; // On target
  }

  private final static double sin45 = Math.sin(Math.toRadians(45));

  private void set(SwerveModule module, double dx, double dy) {
    double speed = Math.sqrt(dx * dx + dy * dy);
    double angle = Math.toDegrees(Math.atan2(dy, dx));

    // if ((Math.abs(dx) >= ArcadeDrive.k_deadZone) || (Math.abs(dy) >=
    // ArcadeDrive.k_deadZone)) {
    // Logger.log("DriveSubsystem", 1, String.format("set: angle=%f, speed = %f,
    // dx=%f, dy=%f", angle, speed, dx, dy));
    // }
    module.setTurnAndSpeed(angle, speed);
  }

  public void drive(double angle, double speed, double turn) {
    // m_FLModule.setTurnPosition(angle);
    // m_FRModule.setTurnPosition(angle);
    // m_RLModule.setTurnPosition(angle);
    // m_RRModule.setTurnPosition(angle);

    // m_FLModule.setDriveSpeed(speed);
    // m_FRModule.setDriveSpeed(speed);
    // m_RLModule.setDriveSpeed(speed);
    // m_RRModule.setDriveSpeed(speed);

    angle = Math.toRadians(angle);
    turn = turn * sin45;
    // turn = 0;
    double dx = speed * Math.cos(angle);
    double dy = speed * Math.sin(angle);
    // Logger.log("DriveSubsystem", 1, String.format("drive: angle=%f, speed=%f,
    // dx=%f, dy=%f", angle, speed, dx, dy));

    set(m_frontLeft, dx + turn, dy + turn);
    set(m_backLeft, dx - turn, dy + turn);
    set(m_backRight, dx - turn, dy - turn);
    set(m_frontRight, dx + turn, dy - turn);

    // m_FLModule.setTurnAndSpeed(angle, speed);
    // m_FRModule.setTurnAndSpeed(angle, speed);
    // m_RLModule.setTurnAndSpeed(angle, speed);
    // m_RRModule.setTurnAndSpeed(angle, speed);
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public static double normalizeAngle(double angle) {
    long time = System.nanoTime();
    angle = angle % 360.0;

    if (angle > 180) {
      angle -= 360;
    } else if (angle < -180) {
      angle += 360;
    }
    time = System.nanoTime() - time;
    // Logger.log("DriveSubsystem", 1, String.format("normalizeAngle: dt=%f", time /
    // 1000000.0));

    return (angle);
  }

  public SwerveModule getFLModule() {
    return m_frontLeft;
  }

  public SwerveModule getBLModule() {
    return m_backLeft;
  }

  public SwerveModule getBRModule() {
    return m_backRight;
  }

  public SwerveModule getFRModule() {
    return m_frontRight;
  }

  // public static Tracer m_tracer = new Tracer();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");

    // Logger.log("DriveSubsystem", 1, String.format("fl angle = %d",
    // m_frontLeft.getRawTurnPosition()));

    // Logger.log("DriveSubsystem", 1, String.format("flp=%f,fla=%d,blp=%f,bla=%d",
    // m_frontLeft.getDrivePosition(),
    // m_frontLeft.getRawTurnPosition(), m_backLeft.getDrivePosition(),
    // m_backLeft.getRawTurnPosition()));

    // Logger.log("DriveSubsystem", 1, String.format(",%f,%f,%f,%f",
    // m_frontLeft.getTurnPosition(), m_backLeft.getTurnPosition(),
    // m_backRight.getTurnPosition(), m_frontRight.getTurnPosition()));

    // m_tracer.clearEpochs();

    // long time = System.currentTimeMillis();
    // Logger.log("DriveSubsystem", 1, "Calling poseEstimator.update()");
    double t = ApriltagsCamera.getTime();
    // m_tracer.addEpoch("ApriltagsCamera.getTime");
    Rotation2d r = getGyroRotation2d();
    // m_tracer.addEpoch("getGyroRotation2d");
    SwerveModulePosition pos[] = getModulePositions();
    // m_tracer.addEpoch("getModulePositions");
    m_poseEstimator.updateWithTime(t, r, pos);
    // m_tracer.addEpoch("poseEstimator.updateWithTime");
    m_camera.processRegions(m_poseEstimator);
    Pose2d pose = m_poseEstimator.getEstimatedPosition();
    m_posServer.setPosition(pose);
    // Logger.log("DriveSubsystem", 1, String.format("x=%f,y=%f,yaw=%f",
    // pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
    // time = System.currentTimeMillis() - time;
    // Logger.log("DriveSubsystem", 1, String.format("periodic: dt=%d", time));

    // if (time >= 10) {
    // m_tracer.printEpochs((x)->System.out.println(x));
    // }

    // Robot.sleep(1000);
  }
}
