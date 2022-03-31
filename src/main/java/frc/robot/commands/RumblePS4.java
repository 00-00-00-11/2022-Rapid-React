// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utility.ControllerRumbleType;
import frc.robot.utility.PS4Utility;

public class RumblePS4 extends CommandBase {

  PS4Controller controller;
  ControllerRumbleType type;
  double intensity;

  public RumblePS4(PS4Controller controller, ControllerRumbleType type, double intensity) {
    this.controller = controller;
    this.type = type;
    this.intensity = intensity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PS4Utility.rumble(controller, type, intensity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PS4Utility.rumble(controller, type, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
