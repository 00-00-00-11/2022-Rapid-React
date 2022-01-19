// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExitTarmac extends CommandBase {
  /** Creates a new ExitTarmac. */
  public ExitTarmac() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_autoSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_autoSubsystem.exitTamac();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_autoSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
