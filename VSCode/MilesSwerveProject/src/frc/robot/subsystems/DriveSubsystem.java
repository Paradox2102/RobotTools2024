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

  private static final double k_frontLeftMinSteeringPower = 0.33;
  private static final double k_backLeftMinSteeringPower = 0.33;
  private static final double k_backRightMinSteeringPower = 0.33;
  private static final double k_frontRightMinsteeringPower = 0.33;
  
  private static final int k_frontLeftSteeringZero = -918;
  private static final int k_backLeftSteeringZero = 214;
  private static final int k_backRightSteeringZero = -826;
  private static final int k_frontRightSteeringZero = -2020;

  // private static final int k_frontLeftSteeringZero = -1200;
  // private static final int k_backLeftSteeringZero = 400;
  // private static final int k_backRightSteeringZero = -1000;
  // private static final int k_frontRightSteeringZero = -2200;

  private static final double k_frontLeftSteeringP = 0.90 / 360;
  private static final double k_backLeftSteeringP = 0.90 / 360;
  private static final double k_backRightSteeringP = 0.80 / 360;
  private static final double k_frontRightSteeringP = 0.90 / 360;

  
  private static final double k_frontLeftSteeringD = 0.01;
  private static final double k_backLeftSteeringD = 0.01;
  private static final double k_backRightSteeringD = 0.01;
  private static final double k_frontRightSteeringD = 0.01;

 static final double k_maxDriveSpeed = 1700;
  private static final double k_frontLeftDriveF = 1.0/k_maxDriveSpeed;
  private static final double k_backLeftDriveF = 1.0/k_maxDriveSpeed;
  private static final double k_backRightDriveF = 1.0/k_maxDriveSpeed;
  private static final double k_frontRightDriveF = 1.0/k_maxDriveSpeed;

  private static final double k_frontLeftDriveP = 0.0001;
  private static final double k_backLeftDriveP = 0.0001;
  private static final double k_backRightDriveP = 0.0001;
  private static final double k_frontRightDriveP = 0.0001;

  private static final double k_frontLeftDriveI = 0.0001;
  private static final double k_backLeftDriveI = 0.0001;
  private static final double k_backRightDriveI = 0.0001;
  private static final double k_frontRightDriveI = 0.0001;

 
  
  private static final double k_frontLeftMinDrivePower = 0.34;
  private static final double k_backLeftMinDrivePower = 0.34;
  private static final double k_backRightMinDrivePower = 0.35;
  private static final double k_frontRightMinDrivePower = 0.35;
 
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
    m_frontLeft.setSteeringMinPower(k_frontLeftMinSteeringPower);
    m_frontRight.setSteeringMinPower(k_frontRightMinsteeringPower);
    m_backLeft.setSteeringMinPower(k_backLeftMinSteeringPower);
    m_backRight.setSteeringMinPower(k_backRightMinSteeringPower);

    m_frontLeft.setSteeringZero(k_frontLeftSteeringZero);
    m_frontRight.setSteeringZero(k_frontRightSteeringZero);
    m_backLeft.setSteeringZero(k_backLeftSteeringZero);
    m_backRight.setSteeringZero(k_backRightSteeringZero);
    
    m_frontLeft.setSteeringPTerm(k_frontLeftSteeringP);
    m_frontRight.setSteeringPTerm(k_frontRightSteeringP);
    m_backLeft.setSteeringPTerm(k_backLeftSteeringP);
    m_backRight.setSteeringPTerm(k_backRightSteeringP);


    m_frontLeft.setSteeringDTerm(k_frontLeftSteeringD);
    m_frontRight.setSteeringDTerm(k_frontRightSteeringD);
    m_backLeft.setSteeringDTerm(k_backLeftSteeringD);
    m_backRight.setSteeringDTerm(k_backRightSteeringD);

    m_frontLeft.setMinDrivePower(k_frontLeftMinDrivePower);
    m_frontRight.setSteeringDTerm(k_frontRightMinDrivePower);
    m_backLeft.setSteeringDTerm(k_backLeftMinDrivePower);
    m_backRight.setSteeringDTerm(k_backRightMinDrivePower);

    m_frontLeft.setDriveMotorFTerm(k_frontLeftDriveF);
    m_frontRight.setDriveMotorFTerm(k_frontRightDriveF);
    m_backLeft.setDriveMotorFTerm(k_backLeftDriveF);
    m_backRight.setDriveMotorFTerm(k_backRightDriveF);

    m_frontLeft.setDriveMotorPTerm(k_frontLeftDriveP);
    m_frontRight.setDriveMotorPTerm(k_frontRightDriveP);
    m_backLeft.setDriveMotorPTerm(k_backLeftDriveP);
    m_backRight.setDriveMotorPTerm(k_backRightDriveP);

    m_frontLeft.setDriveMotorITerm(k_frontLeftDriveI);
    m_frontRight.setDriveMotorITerm(k_frontRightDriveI);
    m_backLeft.setDriveMotorITerm(k_backLeftDriveI);
    m_backRight.setDriveMotorITerm(k_backRightDriveI);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
    // double frontLeftSteeringPos = m_frontLeft.getSteeringPosition();
    // double frontRightSteeringPos = m_frontRight.getSteeringPosition();
    // double backLeftSteeringPos = m_backLeft.getSteeringPosition();
    // double backRightSteeringPos = m_backRight.getSteeringPosition();
    // // -921.000000,-1991.000000,259.000000,-872.000000
    // // Logger.log("GetMinSteeringPower", -1, "execute()");
    // // Logger.log("drivesubsystemmotorpos", 0, String.format(",%f,%f,%f,%f",
    //         // frontLeftSteeringPos, frontRightSteeringPos, backLeftSteeringPos, backRightSteeringPos));
  }

  
  public SwerveModule SwerveModuleGetFrontLeftModule() {
    return m_frontLeft;
  }
  public SwerveModule SwerveModuleGetFrontRightModule() {
    return m_frontRight;
  }
  public SwerveModule SwerveModuleGetBackLeftModule() {
    return m_backLeft;
  }
  public SwerveModule SwerveModuleGetBackRightModule() {
    return m_backRight;
  }

}