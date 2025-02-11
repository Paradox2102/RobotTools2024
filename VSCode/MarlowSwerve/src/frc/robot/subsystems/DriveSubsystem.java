/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick; 
import robotCore.Device;
import robotCore.Encoder;
import robotCore.Gyro;
import robotCore.Logger;


public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   * 
   */
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

  private static final double k_backLeftMinSteeringPower = 0.36;
  private static final double k_backRightMinSteeringPower = 0.39;
  private static final double k_frontRightMinsteeringPower = 0.38;
  private static final double k_frontLeftMinSteeringPower = 0.33;

  private static final int k_frontLeftSteeringZero = -780;
  private static final int k_backLeftSteeringZero = -695;
  private static final int k_backRightSteeringZero = -1830;
  private static final int k_frontRightSteeringZero = -1905;

  private static final double k_frontLeftSteeringP = 1.0 / 360;
  private static final double k_backLeftSteeringP = 0.7 / 360;
  private static final double k_backRightSteeringP = 1.0 / 360;
  private static final double k_frontRightSteeringP = 1.0 / 360;

  private static final double k_frontLeftSteeringD = 0.008;
  private static final double k_backLeftSteeringD = 0.0012; 
  private static final double k_backRightSteeringD = 0.008;
  private static final double k_frontRightSteeringD = 0.010;

  private static final double BLMinPower = .44;
  private static final double BRMinPower = .4;
  private static final double FRMinPower = .3;
  private static final double FLMinPower = .4;

  private static final int MaxSpeed = 2250;

  private static final double k_frontLeftDriveF = .8 / MaxSpeed;
  private static final double k_backLeftDriveF = 1.1 / MaxSpeed;
  private static final double k_backRightDriveF = 1.1 / MaxSpeed;
  private static final double k_frontRightDriveF = .95 / MaxSpeed;
 
  public static final double k_drivePTerm = 0.0008;
  public static final double k_driveITerm = 0.0003;
  public static final double k_driveIZone = 80;

  public static final double k_maxAngularSpeed = 114;

  static final double k_ticksPerMeter = 3645;
  public static final double k_maxDriveSpeedMetersPerSecond = MaxSpeed / k_ticksPerMeter;

  private final Translation2d m_frontLeftLocation = new Translation2d(0.05842, 0.05842);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.05842, 0.05842);
  private final Translation2d m_backRightLocation = new Translation2d(-0.05842, -0.05842);
  private final Translation2d m_frontRightLocation = new Translation2d(0.05842, -0.05842);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_backLeftLocation, m_backRightLocation, m_frontRightLocation);
  
   private final Gyro m_gyro = new Gyro();
 
 public SwerveModule m_frontLeft = new SwerveModule(FLDrivePWM, FLDriveDir, FLDriveEncInt, FLDriveEncDir,
      FLSteeringPWM, FLSteeringDir, FLSteeringEncA, FLSteeringEncB, FLI2CAddr, "FL");
  public SwerveModule m_backLeft = new SwerveModule(BLDrivePWM, BLDriveDir, BLDriveEncInt, BLDriveEncDir,
      BLSteeringPWM, BLSteeringDir, BLSteeringEncA, BLSteeringEncB, BLI2CAddr, "BL");
  public SwerveModule m_backRight = new SwerveModule(BRDrivePWM, BRDriveDir, BRDriveEncInt, BRDriveEncDir,
      BRSteeringPWM, BRSteeringDir, BRSteeringEncA, BRSteeringEncB, BRI2CAddr, "BR");
  public SwerveModule m_frontRight = new SwerveModule(FRDrivePWM, FRDriveDir, FRDriveEncInt, FRDriveEncDir,
      FRSteeringPWM, FRSteeringDir, FRSteeringEncA, FRSteeringEncB, FRI2CAddr, "FR"); 


  public DriveSubsystem() { 
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
    m_backLeft.setZero(k_backLeftSteeringZero);
    m_frontRight.setZero(k_frontRightSteeringZero);
    m_frontLeft.setZero(k_frontLeftSteeringZero);
    m_backRight.setZero(k_backRightSteeringZero);

    
    m_frontLeft.setSteeringMinPower(k_frontLeftMinSteeringPower);
    m_backLeft.setSteeringMinPower(k_backLeftMinSteeringPower);
    m_backRight.setSteeringMinPower(k_backRightMinSteeringPower);
    m_frontRight.setSteeringMinPower(k_frontRightMinsteeringPower);

    m_backRight.setSteeringPTerm(k_backRightSteeringP);
    m_backRight.setSteeringDTerm(k_backRightSteeringD);
    
    m_backLeft.setSteeringPTerm(k_backLeftSteeringP);
    m_backLeft.setSteeringDTerm(k_backLeftSteeringD);

    m_frontRight.setSteeringPTerm(k_frontRightSteeringP);
    m_frontRight.setSteeringDTerm(k_frontRightSteeringD);

    m_frontLeft.setSteeringPTerm(k_frontLeftSteeringP);
    m_frontLeft.setSteeringDTerm(k_frontLeftSteeringD);

    // m_frontLeft.setSteeringPosition(0);
    // m_frontRight.setSteeringPosition(0);
    // m_backLeft.setSteeringPosition(0);
    // m_backRight.setSteeringPosition(0);

    m_frontLeft.setDriveMinPower(FLMinPower);
    m_backLeft.setDriveMinPower(BLMinPower);
    m_backRight.setDriveMinPower(BRMinPower);
    m_frontRight.setDriveMinPower(FRMinPower);

    m_frontLeft.setDriveMaxSpeed(MaxSpeed);
    m_backLeft.setDriveMaxSpeed(MaxSpeed);
    m_backRight.setDriveMaxSpeed(MaxSpeed);
    m_frontRight.setDriveMaxSpeed(MaxSpeed);

    m_frontLeft.setDriveFTerm(k_frontLeftDriveF);
    m_backLeft.setDriveFTerm(k_backLeftDriveF);
    m_frontRight.setDriveFTerm(k_frontRightDriveF);
    m_backRight.setDriveFTerm(k_backRightDriveF);

    m_frontLeft.setDrivePTerm(k_drivePTerm);
    m_backLeft.setDrivePTerm(k_drivePTerm);
    m_frontRight.setDrivePTerm(k_drivePTerm);
    m_backRight.setDrivePTerm(k_drivePTerm);

    m_frontLeft.setDriveITerm(k_driveITerm);
    m_backLeft.setDriveITerm(k_driveITerm);
    m_frontRight.setDriveITerm(k_driveITerm);
    m_backRight.setDriveITerm(k_driveITerm);

    m_frontLeft.setDriveIZone(k_driveIZone);
    m_backLeft.setDriveIZone(k_driveIZone);
    m_frontRight.setDriveIZone(k_driveIZone);
    m_backRight.setDriveIZone(k_driveIZone);

    m_gyro.reset(0);

    







    



  


    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
    Logger.log("DriveSubsystem", -1, String.format( "FL, FR, BL, BR: %f,%f,%f,%f", m_frontLeft.getSteeringPos(),  m_frontRight.getSteeringPos(), m_backLeft.getSteeringPos(), m_backRight.getSteeringPos()));
    
    
  }
  public SwerveModule getFrontleftModule(){
    return m_frontLeft;
  }

  public SwerveModule getFrontRightModule(){
    return m_frontRight;
  }

  public SwerveModule getBackRightModule(){
    return m_backRight;
  }

  public SwerveModule getBackleftModule(){
    return m_backLeft;
  }
  public void SwerveMotorCalibrate(double Power){
    
    m_backLeft.setSteeringPower(Power);
    m_frontRight.setSteeringPower(Power);
    m_backRight.setSteeringPower(Power);
    m_frontLeft.setSteeringPower(Power);

    Logger.log("DriveSubsystem", 3, String.format( "FL, FR, BL, BR: %f,%f,%f,%f,%f", m_frontLeft.getSteeringPos(),  m_frontRight.getSteeringPos(), m_backLeft.getSteeringPos(), m_backRight.getSteeringPos(),Power));

    


   
  }





 
 public void setPower(double BL, double FL, double BR, double FR){
  m_backLeft.Speedset(BL);
  m_frontLeft.Speedset(FL);
  m_backRight.Speedset(BR);
  m_frontRight.Speedset(FR);
 

  


 }

 public void setRotation(int BL, int FL, int BR, int FR){
  m_backLeft.setSteeringPosition(BL);
  m_frontLeft.setSteeringPosition(FL);
  m_backRight.setSteeringPosition(BR);
  m_frontRight.setSteeringPosition(FR);

  


 }
public int getEncFL(){
  return m_frontLeft.getDriveEncoderP();
}
public int getEncBR(){
  return m_backRight.getDriveEncoderP();
}
public int getEncFR(){
  return m_frontRight.getDriveEncoderP();
}
public int getEncBL(){
  return m_backLeft.getDriveEncoderP();


}




public void setSpeed(double BL, double FL, double BR, double FR){
  m_backLeft.Speedset(BL);
  m_frontLeft.Speedset(FL);
  m_backRight.Speedset(BR);
  m_frontRight.Speedset(FR);
}

private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, k_maxDriveSpeedMetersPerSecond);
  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_backLeft.setDesiredState(swerveModuleStates[1]);
  m_backRight.setDesiredState(swerveModuleStates[2]);
  m_frontRight.setDesiredState(swerveModuleStates[3]);
}


  


  
  
  

public void printzero(){
  Logger.log("DriveSubsystem", 3, String.format( "FL, FR, BL, BR: %f,%f,%f,%f", m_frontLeft.getSteeringPos(),  m_frontRight.getSteeringPos(), m_backLeft.getSteeringPos(), m_backRight.getSteeringPos()));
}




public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));
 
    setModuleStates(swerveModuleStates);
  }
}

