// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ZeroElevator extends CommandBase {
  /** Creates a new ZeroElevator. */
  public ZeroElevator() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climberSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_climberSubsystem.elevatorRetract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_climberSubsystem.limitSwitchIsTriggered();
  }
}
