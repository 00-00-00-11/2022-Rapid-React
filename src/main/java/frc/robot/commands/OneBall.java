// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBall extends SequentialCommandGroup {
  /** Creates a new OneBall. */
  public OneBall() {
    addCommands(
      new ExitTarmac(false),
      new ExitTarmac(true),
      new SequentialCommandGroup(
        new InstantCommand(() -> SmartDashboard.putString("AUTO STATUS", "SHOOTING")),
        new IndexerAndShoot()
      )
    );
  }
}
