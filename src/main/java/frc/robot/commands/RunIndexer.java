// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RunIndexer extends CommandBase {
  /** Creates a new IndexerCommand. */
  public RunIndexer() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_IndexerSubsystem.runIndexer(0d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_IndexerSubsystem.runIndexer(Constants.IndexerConstants.indexerSpeed);
    RobotContainer.m_IndexerSubsystem.setIsRunning(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_IndexerSubsystem.runIndexer(0d);
    RobotContainer.m_IndexerSubsystem.setIsRunning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
