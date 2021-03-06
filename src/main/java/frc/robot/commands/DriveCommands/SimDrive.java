// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SimDrive extends CommandBase {

    SlewRateLimiter filter = new SlewRateLimiter(2);

    /** Creates a new SimDrive. */
    public SimDrive() {
        addRequirements(RobotContainer.m_driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //RobotContainer.m_driveSubsystem.tankDriveAuto(-.5,-.5);

        double valetSpeed;

        valetSpeed = 1;
        double leftAxis = RobotContainer.driverGamepad.getLeftX();
        double rightAxis = RobotContainer.driverGamepad.getRightX();
        double r2 = RobotContainer.driverGamepad.getR2Axis();
        double l2 = RobotContainer.driverGamepad.getL2Axis();

        double speed = (r2 - l2) * valetSpeed * (-1);
        // speed = Math.signum(speed) * speed * speed;
        double adjustedSpeed = filter.calculate(speed);
        // SmartDashboard.putNumber("adjustedSpeed", adjustedSpeed);
        // SmartDashboard.putNumber("leftAxis", leftAxis);

        RobotContainer.m_driveSubsystem.curveDrive(adjustedSpeed, leftAxis, false);

        if (Math.abs(rightAxis) > Constants.DriveConstants.DEADZONE) {
            RobotContainer.m_driveSubsystem.tankDriveAuto(rightAxis * Constants.DriveConstants.TURN_SPEED, -rightAxis * Constants.DriveConstants.TURN_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
