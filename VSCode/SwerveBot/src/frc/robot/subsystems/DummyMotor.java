package frc.robot.subsystems;

import robotCore.Encoder;
import robotCore.Logger;
import robotCore.PWMMotor;

public class DummyMotor extends PWMMotor {
	private static int m_nextMotor = 0;
	private int m_motor;

    public DummyMotor(int pwmPin, int dirPin, int i2cAddr) {
		m_motor = m_nextMotor++;
    }

    	
	/** 
	 *  
	 *  @param maxSpeed - Specifies the maximum speed of the motor. Setting this
	 *  				  value allows control of the speed using the range
	 *  				  -1 to +1 similar to when the motor is controlled by power.
	 */
    @Override
	public void setMaxSpeed(double maxSpeed)
	{
		Logger.log("DummyMotor", 1, String.format("%d:setMaxSpeed(%f)", m_motor, maxSpeed));
	}
	
	/** 
	 *  
	 *  @param value - Sets either the power or the speed depending on the current mode.
	 *  				If the motor is in power mode, this should be in the range -1 to +1.
	 *  				If the motor is in speed mode, then this should either be the desired
	 *  				speed, or -1 to +1 if the <strong>maxSpeed</strong> value has been set.
	 */
    @Override
	public void set(double value)
	{
        Logger.log("DummyMotor", 1, String.format("%d:set(%f)", m_motor, value));
	}
	
	/** 
	 *  
	 *  @param encoder - Specifies the feedback device to be used to control the speed.
	 */
    Encoder m_encoder;
    @Override
	public void setFeedbackDevice(Encoder encoder)
	{
        Logger.log("DummyMotor", 1, String.format("%d:setFeedbackDevice(...)", m_motor));
        m_encoder = encoder;
	}
	
	/** 
	 *  
	 *  @return Returns the current feedback device
	 */
    @Override
	public Encoder getFeedbackDevice()
	{
		Logger.log("DummyMotor", 1, String.format("%d:getFeedbackDevice", m_motor));
		return(m_encoder);
	}
	
	/** 
	 *  
	 *  @param mode - Sets the current control mode.
	 */
    @Override
	public void setControlMode(SmartMotorMode mode)
	{
        Logger.log("DummyMotor", 1, String.format("%d:setControlMode(", m_motor) + mode + ")");
	}
	
	/** 
	 *  
	 *  @param f - Specifies the 'feed forward' term for the PID control.
	 */
    @Override
	public void setFTerm(double f)
	{
        Logger.log("DummyMotor", 1, String.format("%d:setFTerm(%f)", m_motor, f));
	}
	
	/** 
	 *  
	 *  @param i - Specifies the integral term for the PID control.
	 */
    @Override
	public void setITerm(double i)
	{
        Logger.log("DummyMotor", 1, String.format("%d:setITerm(%f)", m_motor, i));
	}
	
	/** 
	 *  
	 *  @param p - Specifies the proportional term for the PID control.
	 */
    @Override
	public void setPTerm(double p)
	{
        Logger.log("DummyMotor", 1, String.format("%d:setPTerm(%f)", m_motor, p));
	}
	
	/** 
	 *  
	 *  @param z - Specifies the I Zone term for the PID control. This is the region
	 *  			outside of which, the I term is ignored.
	 */
    @Override
	public void setIZone(double z)
	{
		Logger.log("DummyMotor", 1, String.format("%d:setIZone(%f)", m_motor, z));
	}
	
	/** 
	 *  
	 *  @param invert - If true then the direction of the motor is reversed
	 */
    @Override
	public void setInverted(boolean invert)
	{
		Logger.log("DummyMotor", 1, String.format("%d:setInverted(%b)", m_motor, invert));
	}
	
	/** 
	 *  
	 *  @param d - Specifies the derivative term for the PID control.
	 */
    @Override
	public void setDTerm(double d)
	{
		Logger.log("DummyMotor", 1, String.format("%d:setDTerm(%f)", m_motor, d));
	}
	
	/** 
	 *  
	 *  @param wrap - When using position control this specifies that the position wraps modulo wrap
	 */
    @Override
	public void setWrap(double wrap)
	{
		Logger.log("DummyMotor", 1, String.format("%d:setWrap(%f)", m_motor, wrap));
	}

	/** 
	 *  
	 *  @param minPower - Specifies the minimum power needed to get the motor to run
	 */
    @Override
	public void setMinPower(double minPower)
	{
		Logger.log("DummyMotor", 1, String.format("%d:setMinPower(%f)", m_motor, minPower));
	}

	/** 
	 *  
	 *  @param minDeadZone - For positional control specifies the max error to be considered on target
	 */
    @Override
	public void setDeadZone(double deadZone)
	{
		Logger.log("DummyMotor", 1, String.format("%d:setDeadZone(%f)", m_motor, deadZone));
	}

	/** 
	 *  
	 *  Disables the motor.
	 */

	// @Override
	// public void disable() 
	// {
	// 	Logger.log("DummyMotor", 1, String.format("%d:disable()", m_motor));
	// }

	/** 
	 *  
	 *  Enables the motor.
	 */
	// @Override
	// public void enable() 
	// {
	// 	Logger.log("DummyMotor", 1, String.format("%d:enable()", m_motor));
	// }
}
