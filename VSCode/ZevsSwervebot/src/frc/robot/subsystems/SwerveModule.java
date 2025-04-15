package frc.robot.subsystems;

import robotCore.Encoder;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class SwerveModule {
  
        private final PWMMotor m_driveMotor;
        private final PWMMotor m_steeringMotor;
        private final Encoder m_driveEncoder;
        private final Encoder m_steeringEncoder;
    
        public SwerveModule(int drivePWM, int driveDir, int driveEncInt, int driveEncDir, int steeringPWM, int steeringDir,
                           int steeringEncA, int steeringEncB,  int i2cAddr, String name) {
    
            m_driveMotor = new PWMMotor(drivePWM, driveDir, i2cAddr);
            m_steeringMotor = new PWMMotor(steeringPWM, steeringDir, i2cAddr);
            
            m_steeringEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, steeringEncA, steeringEncB,
            i2cAddr, true);
            m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, driveEncInt, driveEncDir, i2cAddr, true);
    
            m_driveMotor.setFeedbackDevice(m_driveEncoder);
            m_steeringMotor.setFeedbackDevice(m_steeringEncoder);
    
            m_driveMotor.setFeedbackDevice(m_driveEncoder);
            m_steeringMotor.setFeedbackDevice(m_steeringEncoder);
    
        }

        public void setSteeringPower(double power) {
            m_steeringMotor.setControlMode(SmartMotorMode.Power);
            m_steeringMotor.set(power);
        }
    
        public double getSteeringPosition() {
            return m_steeringEncoder.getPosition();
        }
    
    }

