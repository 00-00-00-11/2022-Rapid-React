// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.DriverStation;

public class ExitTarmac extends CommandBase {


  boolean finished = false;

  boolean isForward;

  /** Creates a new ExitTarmac. */
  public ExitTarmac(boolean forward) {
    addRequirements(RobotContainer.m_driveSubsystem);
    isForward = forward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(isForward) {
      if (RobotContainer.m_driveSubsystem.getEncoderPosition() < 0 ) {
        RobotContainer.m_driveSubsystem.tankDriveAuto(0.6, 0.6);
      } else {
        finished = true;
      }
    } else {
      if (Math.abs(RobotContainer.m_driveSubsystem.getEncoderPosition()) < Units.feetToMeters(Constants.AutoConstants.AUTO_DIST)) {
        RobotContainer.m_driveSubsystem.tankDriveAuto(-0.6, -0.6);
      } else {
        RobotContainer.m_driveSubsystem.tankDriveAuto(0, 0);
        finished = true;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.m_driveSubsystem.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || DriverStation.isTeleop();
  }
}
