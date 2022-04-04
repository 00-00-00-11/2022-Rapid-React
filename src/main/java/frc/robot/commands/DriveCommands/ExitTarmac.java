// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ExitTarmac extends CommandBase {

    boolean finished = false;

    double distance = 0.0;

    /** Creates a new ExitTarmac. */
    public ExitTarmac(double distance) {
        addRequirements(RobotContainer.m_driveSubsystem);
        this.distance = distance;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       // RobotContainer.m_driveSubsystem.resetEncoders();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double encoderDistance = RobotContainer.m_driveSubsystem.getEncoderPosition();

        if (Math.abs(Units.metersToInches(encoderDistance)) < distance ) {
            RobotContainer.m_driveSubsystem.tankDriveAuto(0.5, 0.5);
        } else {
            finished = true;
            RobotContainer.m_driveSubsystem.tankDriveAuto(0.0, 0.0);
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_driveSubsystem.tankDrive(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished || DriverStation.isTeleop();
    }
}
