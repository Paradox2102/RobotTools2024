/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class Drive {
        // public static final double kTicksPerMeter = 4319 / 1.67;
        public static final double kTicksPerMeter = 3649 / 1.397; //5842 / 1.45;
        public static final double FLZeroPosition = -60;    //-60.88; //-58.73; //-153.15;
        public static final double BLZeroPosition = -54.98; //29.09;
        public static final double BRZeroPosition = -148.03; //113.0; //123.07; //27.22;
        public static final double FRZeroPosition = -150;   //-146.39; //-65; //-52.58;
        
        public static final double kMaxSpeed = 2300;
        public static final double kDrivePTerm = 0.001;
        public static final double kDriveITerm = 0.0002;
        public static final double kDriveIZone = 200;

        public static final double FLDriveMinPower = 0.33;
        public static final double FLDriveFTerm = 1.0 / Constants.Drive.kMaxSpeed;

        public static final double BLDriveMinPower = 0.34;
        public static final double BLDriveFTerm = 1.0 / Constants.Drive.kMaxSpeed;

        public static final double BRDriveMinPower = 0.33;
        public static final double BRDriveFTerm = 1.0 / Constants.Drive.kMaxSpeed;

        public static final double FRDriveMinPower = 0.33;
        public static final double FRDriveFTerm = 1.0 / Constants.Drive.kMaxSpeed;


        public static final double kFLTurnMinPower = 0.37;
        public static final double kFLTurnPTerm = 1.2 / 360;
        public static final double kFLTurnDTerm = 0.01;
        // public static final double kFLTurnITerm = 0.001;

        public static final double kBLTurnMinPower = 0.26;
        public static final double kBLTurnPTerm = 1.5 / 360;
        public static final double kBLTurnDTerm = 0.01;

        public static final double kBRTurnMinPower = 0.26;
        public static final double kBRTurnPTerm = 1.5 / 360;
        public static final double kBRTurnDTerm = 0.01;

        public static final double kFRTurnMinPower = 0.36;
        public static final double kFRTurnPTerm = 1.5 / 360;
        public static final double kFRTurnDTerm = 0.01;
    }
    // public static final double kFLTurnMinPower = 0.33;
    // public static final double kFLTurnDTerm = 0.005;
}
