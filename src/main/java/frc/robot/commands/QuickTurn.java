// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class QuickTurn extends CommandBase {
  /** Creates a new QuickTurn. */
  private double angle;

  private DriveSubsystem driveSub = RobotContainer.m_driveSubsystem;
  private double target;
  private double degrees;
  private PIDController turnPID;

  public QuickTurn(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    if (angle > 180) {
      this.angle -= 360;
    }
    addRequirements(driveSub);
    turnPID = driveSub.getTurnPID();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = driveSub.getHeading() + angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = turnPID.calculate(driveSub.getHeading(), target);
    MathUtil.clamp(speed, -1, 1);

    driveSub.curveDrive(0, -speed, true);

    SmartDashboard.putNumber("Quick Turn Speed", speed);
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
