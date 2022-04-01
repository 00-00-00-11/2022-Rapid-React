// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class OneBall extends SequentialCommandGroup {

    /** Creates a new OneBall. */
    public OneBall() {
        addCommands(
            // new ExitTarmac(false),
            // new ExitTarmac(true),
            // new SequentialCommandGroup(
            //   new InstantCommand(() -> SmartDashboard.putString("AUTO STATUS", "SHOOTING")),
            //   new IndexerAndShoot()
            // )

            new InstantCommand(() -> {
                RobotContainer.m_driveSubsystem.curveDrive(0, 0.2, true);
            })
                .withTimeout(40)
        );
    }
}
