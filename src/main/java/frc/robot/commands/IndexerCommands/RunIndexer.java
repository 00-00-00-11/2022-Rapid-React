// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RunIndexer extends CommandBase {

    /** Creates a new IndexerCommand. */
    public RunIndexer() {
        addRequirements(RobotContainer.m_indexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_indexerSubsystem.runIndexerWithProximity(0d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.m_indexerSubsystem.runIndexerWithProximity(Constants.IndexerConstants.indexerSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_indexerSubsystem.runIndexerWithProximity(0d);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.m_indexerSubsystem.beltShouldBeRunning() == false;
    }
}
