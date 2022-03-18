// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  public AutoAim() {
    addRequirements(RobotContainer.m_visionSubsystem);
    addRequirements(RobotContainer.m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_visionSubsystem.autoAlignWithGoal(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(RobotContainer.m_shooter_subsystem.getHorizontalErrorToTarget(0)) < Constants.ShooterConstants.ALIGN_THRESHOLD) {
    //   return true;
    // } else {
    //   return false;
    // }

    return false;
  }
}
