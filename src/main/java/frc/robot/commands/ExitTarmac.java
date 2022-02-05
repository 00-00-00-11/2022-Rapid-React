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
    addRequirements(RobotContainer.m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (RobotContainer.m_driveSubsystem.getEncoderPosition()*(0.5*Math.PI/18.0) < 6) //.5 ft = wheel diameter, 18.0 ft = gearbox ratio
    System.out.println("pppppp");
    RobotContainer.m_driveSubsystem.tankDrive(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.m_driveSubsystem.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
