// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurnDegrees extends CommandBase {

  double degrees;
  double initialDegrees;

  /**
   * Turn the robot a set amount of degrees
   * 
   * @param degrees
   */
  public TurnDegrees(double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_driveSubsystem);
    this.degrees = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialDegrees = RobotContainer.m_driveSubsystem.getHeading();
  }

  public boolean isClose() {
    return (RobotContainer.m_driveSubsystem.getHeading() - initialDegrees
        - degrees) < Constants.DriveConstants.turnTolerance ? true : false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // arbitary use of Constant turn multiplier
    double factor = -MathUtil.clamp(
        (RobotContainer.m_driveSubsystem.getHeading() - initialDegrees - degrees)
            * Constants.DriveConstants.turnMultiplier,
        -0.5, 0.5);

    RobotContainer.m_driveSubsystem.curveDrive(0, .7 * factor, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isClose();
  }
}