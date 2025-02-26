/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.beans.Encoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class GetMinSteeringPower extends Command {
    private final DriveSubsystem m_subsystem;
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;
    private final SwerveModule m_frontRight;



    private double m_power = 0;

    /**
     * Creates a new GetMinSteeringPower.
     *
     * @param subsystem The subsystem used by this command.
     */
    public GetMinSteeringPower(DriveSubsystem subsystem) {
        Logger.log("GetMinSteeringPower", 3, "GetMinSteeringPower()");

        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
        m_frontLeft = subsystem.SwerveModuleGetFrontLeftModule();
        m_frontRight = subsystem.SwerveModuleGetFrontRightModule();
        m_backLeft = subsystem.SwerveModuleGetBackLeftModule();
        m_backRight = subsystem.SwerveModuleGetBackRightModule();

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.log("GetMinSteeringPower", 2, "initialize()");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double frontLeftSteeringPos = m_frontLeft.getSteeringPosition();
        double frontRightSteeringPos = m_frontRight.getSteeringPosition();
        double backLeftSteeringPos = m_backLeft.getSteeringPosition();
        double backRightSteeringPos = m_backRight.getSteeringPosition();

        Logger.log("GetMinSteeringPower", -1, "execute()");
        Logger.log("GetMinSteeringPower", 0, String.format(",%f,%f,%f,%f,%f", m_power,
                frontLeftSteeringPos, frontRightSteeringPos, backLeftSteeringPos, backRightSteeringPos));

        m_frontLeft.setSteeringPower(m_power);
        m_frontRight.setSteeringPower(m_power);
        m_backLeft.setSteeringPower(m_power);
        m_backRight.setSteeringPower(m_power);
        m_power += .0025;

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.log("GetMinSteeringPower", 2, String.format("end(%b)", interrupted));
        m_frontLeft.setSteeringPower(0);
        m_frontRight.setSteeringPower(0);
        m_backLeft.setSteeringPower(0);
        m_backRight.setSteeringPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Logger.log("GetMinSteeringPower", -1, "isFinished()");
        return m_power > 1;
    }
}
