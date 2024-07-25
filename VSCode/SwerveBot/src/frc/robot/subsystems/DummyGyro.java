package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import robotCore.Gyro;
import robotCore.Logger;

public class DummyGyro extends Gyro {

    public DummyGyro() {
        super(0);
    }
    
    /**
     * Gets only the yaw.
     *
     * @return Returns the yaw.
     */
    @Override
    public double getYaw() {
        Logger.log("DummyGyro", 1, "getYaw()");
        return 0;
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /**
     * Resets the yaw and position of the robot.
     *
     * @param yaw - Specifies the new yaw value for the robot in degrees. This
     *            value will be adjusted so that it in the range -180 to 180.
     */
    @Override
    public void reset(double yaw) {
        Logger.log("DummyGyro", 1, String.format("reset(%f)", yaw));
    }

    /**
     * Specifies that the sense 
     * of the <b>yaw</b> be inverted. I.E. controls whether
     * the <b>yaw</b> value increases or decreases when the robot turns clockwise.
     * In general, the <b>Pure Pursuit</b> path following assumes that the
     * <b>yaw</b> will
     * decrease when the robot turns clockwise.
     *
     * @param invert - If true, inverts the direction of the yaw
     */
    @Override
    public void invert(boolean invert) {
        Logger.log("DummyGyro", 1, String.format("invert(%b)", invert));
    }
}
