// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ClimberSubsystem;

public class AutoClimberCommand extends CommandBase {
  /** Creates a new AutoClimberCommand. */
  public AutoClimberCommand(ClimberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_climberSubsystem.nextRung();

    if (!RobotContainer.m_climberSubsystem.checkNextRungComplete()) {
      execute();
    } else {
      RobotContainer.m_climberSubsystem.resetSteps();
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
