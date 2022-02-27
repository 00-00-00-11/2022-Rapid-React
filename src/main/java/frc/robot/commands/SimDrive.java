// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class SimDrive extends CommandBase {

  private DriveSubsystem driveSub = RobotContainer.m_driveSubsystem;

  /** Creates a new SimDrive. */
  public SimDrive() {
    // Use addRequirRobments() here to declare subsystem dependencies.
    addRequirements(driveSub);
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
    if (driveSub.getDirection() < 0) {
      leftAxis = -leftAxis;
    }
    driveSub.curveDrive(speed, leftAxis, false);

    if (Math.abs(rightAxis) > .25) { // TODO make .25 a cosntant
      driveSub.curveDrive(0, rightAxis, true); // Will override previous curve drive
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
