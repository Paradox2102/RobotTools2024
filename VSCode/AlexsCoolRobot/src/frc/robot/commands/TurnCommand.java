package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Encoder;
import robotCore.Logger;

public class TurnCommand extends Command {
    private final DriveSubsystem m_subsystem;
    private double m_speed;
    private double m_angle;
    private Encoder m_leftEncoder;
    private Encoder m_rightEncoder;

public TurnCommand(DriveSubsystem subsystem, double speed, double angle) {
    Logger.log("TurnCommand", 3, "TurnCommand()");
    m_subsystem = subsystem;
    m_speed = speed;
    m_angle = angle;
    m_leftEncoder = m_subsystem.getLeftEncoder();
    m_rightEncoder = m_subsystem.getRightEncoder();
    addRequirements(m_subsystem);
}
@Override
public void initialize(){
Logger.log("TurnCommand", 2, "initalize()");
m_leftEncoder.reset();
m_rightEncoder.reset();
if (m_angle < 0)
{
    m_subsystem.setSpeed(-m_speed, m_speed);
}
else
{
    m_subsystem.setSpeed(m_speed, -m_speed);
}
}

@Override
public void execute(){
    Logger.log("TurnCommand", -1, "execute()");
}

@Override
public void end(boolean interrupted) {
    Logger.log("TurnCommand", 2, String.format("end(%b)", interrupted));
    m_subsystem.setPower(0,0);
}

@Override
public boolean isFinished() {
    Logger.log("TurnCommand", -1, "isFinished()");
    int delta = m_leftEncoder.get() - m_rightEncoder.get();
    return (Math.abs(delta) >= Math.abs(m_angle));
}
}
    

