/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.apriltags.ApriltagsCamera;
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

  private static final double k_frontLeftMinSteeringPower = 0.33;
  private static final double k_backLeftMinSteeringPower = 0.33;
  private static final double k_backRightMinSteeringPower = 0.30;
  private static final double k_frontRightMinsteeringPower = 0.30;

  // Blue Robot
  private static final int k_frontLeftSteeringZero = -1950;
  private static final int k_backLeftSteeringZero = -764;
  private static final int k_backRightSteeringZero = -1940;
  private static final int k_frontRightSteeringZero = 1470;

  // White Robot
  // private static final int k_frontLeftSteeringZero = 255;
  // private static final int k_backLeftSteeringZero = 326;
  // private static final int k_backRightSteeringZero = -2019;
  // private static final int k_frontRightSteeringZero = -1965;

  private static final double k_frontLeftSteeringP = 0.8 / 360;
  private static final double k_backLeftSteeringP = 0.8 / 360;
  private static final double k_backRightSteeringP = 0.9 / 360;
  private static final double k_frontRightSteeringP = 0.8 / 360;

  private static final double k_frontLeftSteeringD = 0.005;
  private static final double k_backLeftSteeringD = 0.005;
  private static final double k_backRightSteeringD = 0.005;
  private static final double k_frontRightSteeringD = 0.005;

  // Blue Robot
  private static final double k_frontLeftMinDrivePower = 0.29;
  private static final double k_backLeftMinDrivePower = 0.29;
  private static final double k_backRightMinDrivePower = 0.30;
  private static final double k_frontRightMinDrivePower = 0.30;

  // White Robot
  // private static final double k_frontLeftMinDrivePower = 0.38;
  // private static final double k_backLeftMinDrivePower = 0.38;
  // private static final double k_backRightMinDrivePower = 0.38;
  // private static final double k_frontRightMinDrivePower = 0.38;

  static final double k_maxDriveSpeed = 2500;
  static final double k_ticksPerMeter = 3579 / 1.345;
  public static final double k_maxDriveSpeedMetersPerSecond = k_maxDriveSpeed / k_ticksPerMeter; // = 0.9395 m/s
  public static final double k_maxAngularSpeed = 2.0; // radians / sec

  private static final double k_frontLeftDriveF = 1.05 / k_maxDriveSpeed;
  private static final double k_backLeftDriveF = 1.05 / k_maxDriveSpeed;
  private static final double k_backRightDriveF = 0.95 / k_maxDriveSpeed;
  private static final double k_frontRightDriveF = 0.95 / k_maxDriveSpeed;

  public static final double k_drivePTerm = 0.0010;
  public static final double k_driveITerm = 0.0004;
  public static final double k_driveIZone = 200;

  private final Gyro m_gyro = new Gyro();
  private ApriltagsCamera m_camera = new ApriltagsCamera();
  private final PositionServer m_posServer = new PositionServer(m_camera);

  SwerveModule m_frontLeft = new SwerveModule(FLDrivePWM, FLDriveDir, FLDriveEncInt, FLDriveEncDir,
      FLTurnPWM, FLTurnDir, FLTurnEncA, FLTurnEncB, FLI2CAddr, "FrontLeft");
  SwerveModule m_backLeft = new SwerveModule(BLDrivePWM, BLDriveDir, BLDriveEncInt, BLDriveEncDir,
      BLTurnPWM, BLTurnDir, BLTurnEncA, BLTurnEncB, BLI2CAddr, "BackLeft");
  SwerveModule m_backRight = new SwerveModule(BRDrivePWM, BRDriveDir, BRDriveEncInt, BRDriveEncDir,
      BRTurnPWM, BRTurnDir, BRTurnEncA, BRTurnEncB, BRI2CAddr, "BackRight");
  SwerveModule m_frontRight = new SwerveModule(FRDrivePWM, FRDriveDir, FRDriveEncInt, FRDriveEncDir,
      FRTurnPWM, FRTurnDir, FRTurnEncA, FRTurnEncB, FRI2CAddr, "FrontRight");

  private final Translation2d m_frontLeftLocation = new Translation2d(0.05842, 0.05842);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.05842, 0.05842);
  private final Translation2d m_backRightLocation = new Translation2d(-0.05842, -0.05842);
  private final Translation2d m_frontRightLocation = new Translation2d(0.05842, -0.05842);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_backLeftLocation, m_backRightLocation, m_frontRightLocation);

  private SwerveDrivePoseEstimator m_poseEstimator;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");

    m_frontLeft.setSteeringMinPower(k_frontLeftMinSteeringPower);
    m_backLeft.setSteeringMinPower(k_backLeftMinSteeringPower);
    m_backRight.setSteeringMinPower(k_backRightMinSteeringPower);
    m_frontRight.setSteeringMinPower(k_frontRightMinsteeringPower);

    m_frontLeft.setSteeringZero(k_frontLeftSteeringZero);
    m_backLeft.setSteeringZero(k_backLeftSteeringZero);
    m_backRight.setSteeringZero(k_backRightSteeringZero);
    m_frontRight.setSteeringZero(k_frontRightSteeringZero);

    m_frontLeft.setSteeringPTerm(k_frontLeftSteeringP);
    m_frontLeft.setSteeringDTerm(k_frontLeftSteeringD);

    m_backLeft.setSteeringPTerm(k_backLeftSteeringP);
    m_backLeft.setSteeringDTerm(k_backLeftSteeringD);

    m_backRight.setSteeringPTerm(k_backRightSteeringP);
    m_backRight.setSteeringDTerm(k_backRightSteeringD);

    m_frontRight.setSteeringPTerm(k_frontRightSteeringP);
    m_frontRight.setSteeringDTerm(k_frontRightSteeringD);

    m_frontLeft.setDriveMinPower(k_frontLeftMinDrivePower);
    m_backLeft.setDriveMinPower(k_backLeftMinDrivePower);
    m_backRight.setDriveMinPower(k_backRightMinDrivePower);
    m_frontRight.setDriveMinPower(k_frontRightMinDrivePower);

    m_frontLeft.setDriveF(k_frontLeftDriveF);
    m_backLeft.setDriveF(k_backLeftDriveF);
    m_backRight.setDriveF(k_backRightDriveF);
    m_frontRight.setDriveF(k_frontRightDriveF);

    m_frontLeft.setDriveP(k_drivePTerm);
    m_backLeft.setDriveP(k_drivePTerm);
    m_backRight.setDriveP(k_drivePTerm);
    m_frontRight.setDriveP(k_drivePTerm);

    m_frontLeft.setDriveI(k_driveITerm);
    m_backLeft.setDriveI(k_driveITerm);
    m_backRight.setDriveI(k_driveITerm);
    m_frontRight.setDriveI(k_driveITerm);

    m_frontLeft.setDriveIZone(k_driveIZone);
    m_backLeft.setDriveIZone(k_driveIZone);
    m_backRight.setDriveIZone(k_driveIZone);
    m_frontRight.setDriveIZone(k_driveIZone);

    m_gyro.invert(true);
    m_gyro.reset(0);

    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), getModulePositions(),
        new Pose2d(1, 3, Rotation2d.fromDegrees(0)));

    m_posServer.start();

  }

  private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, k_maxDriveSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_backLeft.setDesiredState(swerveModuleStates[1]);
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_frontRight.setDesiredState(swerveModuleStates[3]);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };
  }

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
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));

    // Logger.log("DriveSubsystem", 1,
    // String.format("xs=%f,ys=%f,m0.angle=%f", xSpeed, ySpeed,
    // swerveModuleStates[0].angle.getDegrees()));

    setModuleStates(swerveModuleStates);
  }

  public void stop() {
    m_frontLeft.stop();
    m_backLeft.stop();
    m_backRight.stop();
    m_frontRight.stop();
  }

  public SwerveModule getFrontLeftModule() {
    return m_frontLeft;
  }

  public SwerveModule getBackLeftModule() {
    return m_backLeft;
  }

  public SwerveModule getBackRightModule() {
    return m_backRight;
  }

  public SwerveModule getFrontRighModule() {
    return m_frontRight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");

    m_poseEstimator.updateWithTime(ApriltagsCamera.getTime(), m_gyro.getRotation2d(), getModulePositions());
    m_posServer.setPosition(m_poseEstimator.getEstimatedPosition());

    // Logger.log("DriveSubsystem", 1, String.format(",%f,%f,%f,%f",
    // m_frontLeft.getSteeringPosition(),
    // m_backLeft.getSteeringPosition(), m_backRight.getSteeringPosition(),
    // m_frontRight.getSteeringPosition()));

    // Logger.log("DriveSubsystem", 1, String.format(",%f,%f,%f,%f",
    // m_frontLeft.getSteeringPositionInDegrees(),
    // m_backLeft.getSteeringPositionInDegrees(),
    // m_backRight.getSteeringPositionInDegrees(),
    // m_frontRight.getSteeringPositionInDegrees()));
  }
}
