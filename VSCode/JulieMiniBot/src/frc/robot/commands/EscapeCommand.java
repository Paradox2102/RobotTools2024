/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;
 
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.DigitalInput;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class EscapeCommand extends Command {
    private final DriveSubsystem m_subsystem;
    private final DigitalInput m_irSensor;
    private final Timer m_timer = new Timer();
    private static final double k_backingTime = 0.5;
    private static final double k_turnSpeed = 0.4;
    private static final double k_minTurnTime = 0.5;
    private double m_turnTime;

    private enum State {
        DriveForward, BackUp, Turn
    }

    private State m_state;

    private static final double k_driveSpeed = 0.4;

    /**
     * Creates a new EscapeCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public EscapeCommand(DriveSubsystem subsystem, DigitalInput irSensor) {
        Logger.log("EscapeCommand", 3, "EscapeCommand()");
 
        m_subsystem = subsystem;
        m_irSensor = irSensor;

        m_timer.reset();
        m_timer.start();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    private void driveForward() {
        Logger.log("DriveToLineCommand", 2, "driveForward()");
    
        m_state = State.DriveForward;
        
        m_subsystem.setSpeed(k_driveSpeed, k_driveSpeed);
    }

    private void drivingForward() {
        if (m_irSensor.get()) {
            Logger.log("EscapeCommand", 2, "BackUp");

            m_state = State.BackUp;

            m_subsystem.setSpeed(-k_driveSpeed, -k_driveSpeed);

            m_timer.reset();
        }
    }

    private void backing() {
        if (m_timer.get() >= k_backingTime) {
            Logger.log("EsapeCommand", 2, "Turn");
        
            m_state = State.Turn;
    
            if ((int)Math.random() == 1) {
                m_subsystem.setSpeed(k_turnSpeed, -k_turnSpeed); // Turn right
            } else {
                m_subsystem.setSpeed(-k_turnSpeed, k_turnSpeed); // Turn left
            }
        
            m_turnTime = k_minTurnTime + Math.random();
    
            m_timer.reset();
        }
    }

    protected void turning() {
        if (m_timer.get() >= m_turnTime) {
            driveForward();
        }
    }
    
 
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.log("EscapeCommand", 2, "initialize()");

        driveForward();
    }
 
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.log("EscapeCommand", -1, "execute()");

        
        switch (m_state) {
            case DriveForward:
                drivingForward();
                break;
    
            case BackUp:
                backing();
                break;
    
            case Turn:
                turning();
                break;
        }
    }
 
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.log("EscapeCommand", 2, String.format("end(%b)", interrupted));
    }
 
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Logger.log("EscapeCommand", -1, "isFinished()");
        return false;
    }
}