// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChainedCommands.*;;

public class OneBall extends SequentialCommandGroup {
  /** Creates a new OneBall. */
  public OneBall() {
    addCommands(
        new ExitTarmac(65), 

        
        new ParallelRaceGroup(
            new IndexerAndShoot(),
            new WaitCommand(5.0)
        ),
        
        new AutoMoveBack()
    );
  }
}
