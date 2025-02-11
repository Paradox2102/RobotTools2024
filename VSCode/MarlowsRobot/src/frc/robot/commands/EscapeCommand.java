/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.DigitalInput;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class EscapeCommand extends Command {
  private enum state { Forward, Backward, Turn}
  private final DriveSubsystem m_subsystem;
  private final DigitalInput m_irSensor;
  private static final double k_driveSpeed = 0.4;
  private final Timer m_timer = new Timer();
  private final int k_backingtime = 2;
  private final double k_turnSpeed = 0.3;
  private final double k_minturntimer = 0.3;
  private Random m_random = new Random(System.currentTimeMillis());
  private double k_turntimer;
  private state m_state;

  /**
   * Creates a new EscapeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EscapeCommand(DriveSubsystem subsystem, DigitalInput irSensor ) {
    Logger.log("EscapeCommand", 3, "EscapeCommand()");
    m_timer.start();
    m_subsystem = subsystem;
    m_irSensor = irSensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("EscapeCommand", 2, "initialize()");
    m_timer.reset();
    Driveforward();
  }

  public void Driveforward() {
    Logger.log("EscapeCommand", 2, "initialize()");
    m_state = state.Forward;
    m_subsystem.setPower(k_driveSpeed,k_driveSpeed);
    
    
  } 
  public void DrivingForward(){
    if (m_irSensor.get()){
    m_state = state.Backward;

    m_subsystem.setPower(-k_driveSpeed, -k_driveSpeed); 

    m_timer.reset();


  }}
  public void Turn() {
    Logger.log("EscapeCommand", 2, "initialize()");
    if(m_timer.get() >= k_turntimer){ Driveforward();}

  }
  public void Backward() {
    Logger.log("EscapeCommand", 2, "initialize()");
    if(m_timer.get() >= k_backingtime){
      if (m_random.nextInt(2) == 1) {
        m_subsystem.setSpeed(k_turnSpeed, -k_turnSpeed); // Turn right
      } else {
        m_subsystem.setSpeed(-k_turnSpeed, k_turnSpeed); // Turn left
      }
      k_turntimer = k_minturntimer + m_random.nextDouble();
      m_timer.reset();
    m_state = state.Turn;


    }

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("EscapeCommand", -1, "execute()");
    switch(m_state) {
      case Forward: 
        DrivingForward();
       break;
      case Backward:
       Backward();
       break;
      case Turn:
       Turn();
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
    return (m_timer.get() >= 7);
  }
}
