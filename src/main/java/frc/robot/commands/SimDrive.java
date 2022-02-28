// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SimDrive extends CommandBase {

  /** Creates a new SimDrive. */
  public SimDrive() {
    // Use addRequirRobments() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean valet = SmartDashboard.getBoolean("Valet Mode", false);

    double valetSpeed;

    valetSpeed = valet ? .3 : 1;

    double leftAxis = RobotContainer.driverGamepad.getLeftX();
    double rightAxis = RobotContainer.driverGamepad.getRightX();
    double r2 = RobotContainer.driverGamepad.getR2Axis();
    double l2 = RobotContainer.driverGamepad.getL2Axis();

    double speed = (r2 - l2) * valetSpeed;
    RobotContainer.m_driveSubsystem.curveDrive(speed, leftAxis, false);

    if (Math.abs(rightAxis) > Constants.DriveConstants.DEADZONE) { 
      RobotContainer.m_driveSubsystem.curveDrive(0, rightAxis, true); 
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
