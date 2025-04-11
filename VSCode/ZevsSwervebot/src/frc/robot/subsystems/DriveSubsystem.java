package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Logger;

public class DriveSubsystem extends SubsystemBase {
  public static final int FLI2CAddr = 5;
  public static final int FLDrivePWM = Device.M1_1_PWM;
  public static final int FLDriveDir = Device.M1_1_DIR;
  public static final int FLSteeringPWM = Device.M1_2_PWM;
  public static final int FLSteeringDir = Device.M1_2_DIR;
  public static final int FLDriveEncInt = Device.Q1_INT;
  public static final int FLDriveEncDir = Device.Q1_DIR;
  public static final int FLSteeringEncA = Device.A1_A;
  public static final int FLSteeringEncB = Device.A1_B;

  public static final int BLI2CAddr = 5;
  public static final int BLDrivePWM = Device.M2_1_PWM;
  public static final int BLDriveDir = Device.M2_1_DIR;
  private static final int BLSteeringPWM = Device.M2_2_PWM;
  private static final int BLSteeringDir = Device.M2_2_DIR;
  public static final int BLDriveEncInt = Device.Q2_INT;
  public static final int BLDriveEncDir = Device.Q2_DIR;
  public static final int BLSteeringEncA = Device.A2_A;
  public static final int BLSteeringEncB = Device.A2_B;

  public static final int BRI2CAddr = 6;
  public static final int BRDrivePWM = Device.M1_1_PWM;
  public static final int BRDriveDir = Device.M1_1_DIR;
  public static final int BRSteeringPWM = Device.M1_2_PWM;
  public static final int BRSteeringDir = Device.M1_2_DIR;
  private static final int BRDriveEncInt = Device.Q1_INT;
  private static final int BRDriveEncDir = Device.Q1_DIR;
  private static final int BRSteeringEncA = Device.A1_A;
  private static final int BRSteeringEncB = Device.A1_B;

  private static final int FRI2CAddr = 6;
  private static final int FRDrivePWM = Device.M2_1_PWM;
  private static final int FRDriveDir = Device.M2_1_DIR;
  private static final int FRSteeringPWM = Device.M2_2_PWM;
  private static final int FRSteeringDir = Device.M2_2_DIR;
  private static final int FRDriveEncInt = Device.Q2_INT;
  private static final int FRDriveEncDir = Device.Q2_DIR;
  private static final int FRSteeringEncA = Device.A2_A;
  private static final int FRSteeringEncB = Device.A2_B;

  SwerveModule m_frontLeft = new SwerveModule(FLDrivePWM, FLDriveDir, FLDriveEncInt, FLDriveEncDir,
      FLSteeringPWM, FLSteeringDir, FLSteeringEncA, FLSteeringEncB, FLI2CAddr, "FrontLeft");
  SwerveModule m_backLeft = new SwerveModule(BLDrivePWM, BLDriveDir, BLDriveEncInt, BLDriveEncDir,
      BLSteeringPWM, BLSteeringDir, BLSteeringEncA, BLSteeringEncB, BLI2CAddr, "BackLeft");
  SwerveModule m_backRight = new SwerveModule(BRDrivePWM, BRDriveDir, BRDriveEncInt, BRDriveEncDir,
      BRSteeringPWM, BRSteeringDir, BRSteeringEncA, BRSteeringEncB, BRI2CAddr, "BackRight");
  SwerveModule m_frontRight = new SwerveModule(FRDrivePWM, FRDriveDir, FRDriveEncInt, FRDriveEncDir,
      FRSteeringPWM, FRSteeringDir, FRSteeringEncA, FRSteeringEncB, FRI2CAddr, "FrontRight");

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}