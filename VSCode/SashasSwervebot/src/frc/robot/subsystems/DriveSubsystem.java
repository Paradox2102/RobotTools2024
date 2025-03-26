/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import robotCore.Device;
import robotCore.Logger;

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

  private static final double k_frontLeftMinSteeringPower = 0.3225;
  private static final double k_backLeftMinSteeringPower = 0.4175;
  private static final double k_backRightMinSteeringPower = 0.42;
  private static final double k_frontRightMinsteeringPower = 0.42;

  private static final int k_frontLeftSteeringZero = -1742;
  private static final int k_backLeftSteeringZero = -633;
  private static final int k_backRightSteeringZero = -1850;
  private static final int k_frontRightSteeringZero = 392;

  //Back Right P and D are set and good. (1/360 and .009 at time of writing)
  private static final double k_BRsteeringPTerm = 1.0/360;
  private static final double k_BRsteeringDTerm = 0.009;
  //Front Right P and D are set and good. (1/360 and .009 at time of writing)
  private static final double k_FRsteeringPTerm = 1.0/360;
  private static final double k_FRsteeringDTerm = 0.009;
  //Back Left P and D are set and good. (1/550 and .0085 at time of writing)
  private static final double k_BLsteeringPTerm = 1.0/550;
  private static final double k_BLsteeringDTerm = 0.01;
  //Front Left P and D are set and good. (1/360 and .009 at time of writing)
  private static final double k_FLsteeringPTerm = 1.0/360;
  private static final double k_FLsteeringDTerm = 0.009;

  private static final double k_maxDriveSpeed = 2500;

  //Back Right F is NOT set and good. ( at time of writing)
  private static final double k_BRDriveFTerm = 1/k_maxDriveSpeed;
  //Front Right F is NOT set and good. ( at time of writing)
  private static final double k_FRDriveFTerm = 1/k_maxDriveSpeed;
  //Back Left F is NOT set and good. ( at time of writing)
  private static final double k_BLDriveFTerm = 1/k_maxDriveSpeed;
  //Front Left F is NOT set and good. ( at time of writing)
  private static final double k_FLDriveFTerm = 1/k_maxDriveSpeed;

  public static final double k_FLdrivePTerm = 0.0001;
  public static final double k_FRdrivePTerm = 0.0001;
  public static final double k_BLdrivePTerm = 0.00005;
  public static final double k_BRdrivePTerm = 0.00005;

  public static final double k_FLdriveITerm = 0.0005;
  public static final double k_FRdriveITerm = 0.0003;
  public static final double k_BLdriveITerm = 0.0005;
  public static final double k_BRdriveITerm = 0.0006;

  public static final double k_driveIZone = 200;

  private static final double k_frontLeftMinDrivePower = 0.25;
  private static final double k_backLeftMinDrivePower = 0.25;
  private static final double k_backRightMinDrivePower = 0.25;
  private static final double k_frontRightMinDrivePower = 0.25;

  

  SwerveModule m_frontLeft = new SwerveModule(FLDrivePWM, FLDriveDir, FLDriveEncInt, FLDriveEncDir,
      FLTurnPWM, FLTurnDir, FLTurnEncA, FLTurnEncB, FLI2CAddr, "FrontLeft");
  SwerveModule m_backLeft = new SwerveModule(BLDrivePWM, BLDriveDir, BLDriveEncInt, BLDriveEncDir,
      BLTurnPWM, BLTurnDir, BLTurnEncA, BLTurnEncB, BLI2CAddr, "BackLeft");
  SwerveModule m_backRight = new SwerveModule(BRDrivePWM, BRDriveDir, BRDriveEncInt, BRDriveEncDir,
      BRTurnPWM, BRTurnDir, BRTurnEncA, BRTurnEncB, BRI2CAddr, "BackRight");
  SwerveModule m_frontRight = new SwerveModule(FRDrivePWM, FRDriveDir, FRDriveEncInt, FRDriveEncDir,
      FRTurnPWM, FRTurnDir, FRTurnEncA, FRTurnEncB, FRI2CAddr, "FrontRight");

  public SwerveModule getFrontLeftModule() {
    return m_frontLeft;
  }
  public SwerveModule getBackLeftModule() {
    return m_backLeft;
  }
  public SwerveModule getFrontRightModule() {
    return m_frontRight;
  }
  public SwerveModule getBackRightModule() {
    return m_backRight;
  }
  
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
    m_frontLeft.setSteeringMinPower(k_frontLeftMinSteeringPower);
    m_backLeft.setSteeringMinPower(k_backLeftMinSteeringPower);
    m_backRight.setSteeringMinPower(k_backRightMinSteeringPower);
    m_frontRight.setSteeringMinPower(k_frontRightMinsteeringPower);

    m_frontLeft.setDriveMinPower(k_frontLeftMinDrivePower);
    m_backLeft.setDriveMinPower(k_backLeftMinDrivePower);
    m_backRight.setDriveMinPower(k_backRightMinDrivePower);
    m_frontRight.setDriveMinPower(k_frontRightMinDrivePower);

    m_frontLeft.setSteeringZero(k_frontLeftSteeringZero);
    m_backLeft.setSteeringZero(k_backLeftSteeringZero);
    m_backRight.setSteeringZero(k_backRightSteeringZero);
    m_frontRight.setSteeringZero(k_frontRightSteeringZero);

    m_frontLeft.setSteeringPTerm(k_BRsteeringPTerm);
    m_backLeft.setSteeringPTerm(k_BLsteeringPTerm);
    m_backRight.setSteeringPTerm(k_FRsteeringPTerm);
    m_frontRight.setSteeringPTerm(k_FLsteeringPTerm);
    
    m_frontLeft.setSteeringDTerm(k_BRsteeringDTerm);
    m_backLeft.setSteeringDTerm(k_BLsteeringDTerm);
    m_backRight.setSteeringDTerm(k_FRsteeringDTerm);
    m_frontRight.setSteeringDTerm(k_FLsteeringDTerm);

    m_frontLeft.setDrivingFTerm(k_BLDriveFTerm);
    m_backLeft.setDrivingFTerm(k_FLDriveFTerm);
    m_backRight.setDrivingFTerm(k_BRDriveFTerm);
    m_frontRight.setDrivingFTerm(k_FRDriveFTerm);

    m_frontLeft.setDrivingPTerm(k_FLdrivePTerm);
    m_frontLeft.setDrivingITerm(k_FLdriveITerm);
    m_frontLeft.setDrivingIZone(k_driveIZone);

    m_frontRight.setDrivingPTerm(k_FRdrivePTerm);
    m_frontRight.setDrivingITerm(k_FRdriveITerm);
    m_frontRight.setDrivingIZone(k_driveIZone);

    m_backRight.setDrivingPTerm(k_BRdrivePTerm);
    m_backRight.setDrivingITerm(k_BRdriveITerm);
    m_backRight.setDrivingIZone(k_driveIZone);

    m_backLeft.setDrivingPTerm(k_BLdrivePTerm);
    m_backLeft.setDrivingITerm(k_BLdriveITerm);
    m_backLeft.setDrivingIZone(k_driveIZone);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}