package frc.robot.subsystems;

import robotCore.Encoder;
import robotCore.Logger;

public class DummyEncoder extends Encoder {
	private static int m_nextEncoder = 0;
	private int m_encoder;

	public DummyEncoder(EncoderType type, int pin1, int pin2, int i2cAddr) {
		Logger.log("DummyEncoder", 1, "Encoder(" + type + String.format(",%d,%d,%d", pin1, pin2, i2cAddr));
		m_encoder = m_nextEncoder++;
	}
	/**
	 * Gets the current encoder value
	 * 
	 * @return Returns the current position or speed. In the case of a Rotational
	 *         encoder the angle is returned in hundredths of a degree
	 */
    @Override
	public int get() {
        Logger.log("DummyEncoder", 1, String.format("%d:get()", m_encoder));
		return 0;
	}

	/**
	 * Gets the current encoder value
	 * 
	 * @return Returns the current position
	 */
    @Override
	public int getPosition() {
        Logger.log("DummyEncoder", 1, String.format("%d:getPosition()", m_encoder));
		return 0;
	}

	/**
	 * Gets the current encoder value
	 * 
	 * @return Returns the current speed
	 */
    @Override
	public int getSpeed() {
        Logger.log("DummyEncoder", 1, String.format("%d:getSpeed()", m_encoder));
		return 0;
	}

	/**
	 * Resets the encoder to zero
	 */
    @Override
	public void reset() {
		Logger.log("DummyEncoder", 1, String.format("%d:reset()", m_encoder));
	}

    // @Override
	// public void setZero(int zero)
	// {
	// 	Logger.log("DummyEncoder", 1, String.format("%d:setZero(%d)", m_encoder, zero));
	// }

	/**
	 * Resets the encoder to the specified position.
	 * 
	 * @param position - Specifies the position to set. In the case of the
	 *                 Rotational encoder this should be the angle in hundredths of
	 *                 a degree.
	 */
    @Override
	public void reset(int position) {
		Logger.log("DummyEncoder", 1, String.format("%d:reset(%d)", m_encoder, position));
	}

	/**
	 * @param invert = If True, the count and speed directions are reversed
	 * 
	 */
    @Override
	public void setInverted(boolean invert) {
        Logger.log("DummyEncoder", 1, String.format("%d, setInverted(%b)", m_encoder, invert));
	}
}
