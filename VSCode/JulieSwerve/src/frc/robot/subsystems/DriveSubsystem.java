/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  

  SwerveModule m_frontLeft = new SwerveModule(FLDrivePWM, FLDriveDir, FLDriveEncInt, FLDriveEncDir,
    FLTurnPWM, FLTurnDir, FLTurnEncA, FLTurnEncB, FLI2CAddr, "FrontLeft");
  SwerveModule m_backLeft = new SwerveModule(BLDrivePWM, BLDriveDir, BLDriveEncInt, BLDriveEncDir,
    BLTurnPWM, BLTurnDir, BLTurnEncA, BLTurnEncB, BLI2CAddr, "BackLeft");
  SwerveModule m_backRight = new SwerveModule(BRDrivePWM, BRDriveDir, BRDriveEncInt, BRDriveEncDir,
    BRTurnPWM, BRTurnDir, BRTurnEncA, BRTurnEncB, BRI2CAddr, "BackRight");
  SwerveModule m_frontRight = new SwerveModule(FRDrivePWM, FRDriveDir, FRDriveEncInt, FRDriveEncDir,
    FRTurnPWM, FRTurnDir, FRTurnEncA, FRTurnEncB, FRI2CAddr, "FrontRight");

    public enum ModulePosition {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
  }

  public SwerveModule getModule(ModulePosition pos) {
    switch (pos) {
      case FRONT_LEFT:
        return m_frontLeft;
      case FRONT_RIGHT:
        return m_frontRight;
      case BACK_LEFT:
        return m_backLeft;
      case BACK_RIGHT:
        return m_backRight;
      default:
        Logger.log("DriveSubsytem", 100, "getModule supplied bad enum, returning m_frontLeft instead.");
        return m_frontLeft;
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
