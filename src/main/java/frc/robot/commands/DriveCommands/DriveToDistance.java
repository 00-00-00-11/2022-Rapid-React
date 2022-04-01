// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveToDistance extends CommandBase {

  double distance = 0;

  public DriveToDistance(double distance) {
    this.distance = distance;
  }

  @Override
  public void initialize() {
    RobotContainer.m_driveSubsystem.resetEncoders();
  }

  @Override
  public void execute() {
    if(distance > 0) {
      if(Math.abs(RobotContainer.m_driveSubsystem.getEncoderPosition()) < distance) {
        RobotContainer.m_driveSubsystem.tankDriveAuto(Constants.DriveConstants.AUTO_SPEED, Constants.DriveConstants.AUTO_SPEED);
      } else {
        RobotContainer.m_driveSubsystem.tankDriveAuto(0,0);
      }
    } else if (distance < 0) {
      if(Math.abs(RobotContainer.m_driveSubsystem.getEncoderPosition()) < distance) {
        RobotContainer.m_driveSubsystem.tankDriveAuto(Constants.DriveConstants.AUTO_SPEED, Constants.DriveConstants.AUTO_SPEED);
      } else {
        RobotContainer.m_driveSubsystem.tankDriveAuto(0,0);
      }
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
