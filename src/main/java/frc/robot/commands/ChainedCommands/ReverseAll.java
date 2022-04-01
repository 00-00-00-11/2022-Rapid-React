// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ChainedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ReverseAll extends CommandBase {
  /** Creates a new ReverseAll. */
  public ReverseAll() {
    addRequirements(RobotContainer.m_indexerSubsystem);
    addRequirements(RobotContainer.m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_indexerSubsystem.runIndexer(-Constants.IndexerConstants.indexerSpeed);
    RobotContainer.m_shooterSubsystem.spinIntake(-Constants.IntakeConstants.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_indexerSubsystem.runIndexer(0);
    RobotContainer.m_shooterSubsystem.spinIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
