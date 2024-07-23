/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Encoder;
import robotCore.Logger;

public class DriveSubsystem extends SubsystemBase {
  public static final int FLTurnEncA = Device.A1_A;
  public static final int FLTurnEncB = Device.A1_B;
  private static final int FRDriveEncInt = Device.Q1_INT;
  private static final int FRDriveEncDir = Device.Q1_DIR;

  private final Encoder m_turnEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, FLTurnEncA,
      FLTurnEncB, 5, false);
  private final Encoder m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, FRDriveEncInt,
      FRDriveEncDir, 5, false);

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
    Logger.log("DriveSubsystem", 1, String.format("Turn Position,%d", m_turnEncoder.getPosition()));
    // Logger.log("DriveSubsystem", 1, String.format("Drive Speed,%d", m_driveEncoder.getSpeed()));
  }
}
