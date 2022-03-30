// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RunIndexerTransition extends CommandBase {
  /** Creates a new IndexerCommand. */
  public RunIndexerTransition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   RobotContainer.m_indexerSubsystem.runIndexerTransition(0d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_indexerSubsystem.runIndexerTransition(Constants.IndexerConstants.indexerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_indexerSubsystem.runIndexerTransition(0d);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
